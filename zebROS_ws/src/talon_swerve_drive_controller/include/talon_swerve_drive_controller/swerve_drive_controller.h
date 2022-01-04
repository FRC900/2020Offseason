/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, PAL Robotics, S.L.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the PAL Robotics nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*
 * Original Author: Enrique Fernández
 */

#ifndef SWERVE_DRIVE_CONTROLLER_INC__
#define SWERVE_DRIVE_CONTROLLER_INC__

#include <array>
#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <controller_interface/controller.h>
#include <talon_controllers/talon_controller_interface.h>

#include <talon_swerve_drive_controller/speed_limiter.h>
#include "talon_swerve_drive_controller/SetXY.h"
#include <talon_swerve_drive_controller/Swerve.h>

#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/tfMessage.h>
#include <tf2_ros/transform_broadcaster.h>

#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

#include <Eigen/Dense>

namespace talon_swerve_drive_controller
{

/**
 * This class makes some assumptions on the model of the robot:
 *  - the rotation axes of wheels are collinear
 *  - the wheels are identical in radius
 * Additional assumptions to not duplicate information readily available in the URDF:
 *  - the wheels have the same parent frame
 *  - a wheel collision geometry is a cylinder or sphere in the urdf
 *  - a wheel joint frame center's vertical projection on the floor must lie within the contact patch
*/
template <size_t WHEELCOUNT>
class TalonSwerveDriveController
	: public controller_interface::Controller<hardware_interface::TalonCommandInterface>
{
	public:
		TalonSwerveDriveController();

		/**
		 * \brief Initialize controller
		 * \param hw            Velocity joint interface for the wheels
		 * \param root_nh       Node handle at root namespace
		 * \param controller_nh Node handle inside the controller namespace
		 */
		bool init(hardware_interface::TalonCommandInterface *hw,
				  ros::NodeHandle &root_nh,
				  ros::NodeHandle &controller_nh) override;

		/**
		 * \brief Updates controller, i.e. computes the odometry and sets the new velocity commands
		 * \param time   Current time
		 * \param period Time since the last called to update
		 */
		void update(const ros::Time &time, const ros::Duration &period) override;

		/**
		 * \brief Starts controller
		 * \param time Current time
		 */
		void starting(const ros::Time &time) override;

		/**
		 * \brief Stops controller
		 * \param time Current time
		 */
		void stopping(const ros::Time & /*time*/) override;

	private:

		bool comp_odom_;
		Eigen::Matrix2Xd wheel_pos_;
		Eigen::Vector2d neg_wheel_centroid_;
		std::array<double, WHEELCOUNT> last_wheel_rot_;	    // used for odom calcs
		std::array<double, WHEELCOUNT> last_wheel_angle_;	//
		double angle_midpoint(double start_angle, double end_angle) const;

		std::string name_;

		std::unique_ptr<swerve<WHEELCOUNT>> swerveC_;

		/// Hardware handles:
		std::array<talon_controllers::TalonControllerInterface, WHEELCOUNT> speed_joints_;
		std::array<talon_controllers::TalonControllerInterface, WHEELCOUNT> steering_joints_;
		/// Velocity command related:

		struct Commands
		{
			Eigen::Vector2d lin;
			double ang;
			ros::Time stamp;

			Commands() : lin(
			{
				0.0, 0.0
			}), ang(0.0), stamp(0.0) {}
		};

		void compOdometry(const ros::Time &time, const double inv_delta_t, const std::array<double, WHEELCOUNT> &steer_angles);

		// True if running cmd_vel, false if running profile
		std::atomic<bool> dont_set_angle_mode_;
		std::atomic<bool> percent_out_drive_mode_;

		realtime_tools::RealtimeBuffer<Commands> command_;
		double brake_last_{ros::Time::now().toSec()};
		double time_before_brake_{0};
		double parking_config_time_delay_{0.1};
		double drive_speed_time_delay_{0.1};

		ros::Subscriber sub_command_;

		ros::ServiceServer change_center_of_rotation_serv_;
		realtime_tools::RealtimeBuffer<Eigen::Vector2d> center_of_rotation_;
		ros::ServiceServer brake_serv_;
		ros::ServiceServer dont_set_angle_mode_serv_;
		ros::ServiceServer percent_out_drive_mode_serv_;

		ros::ServiceServer reset_odom_serv_;
		realtime_tools::RealtimeBuffer<bool> reset_odom_;

		/// Publish executed commands
		std::unique_ptr<realtime_tools::RealtimePublisher<geometry_msgs::TwistStamped> > cmd_vel_pub_;

		/// Wheel radius (assuming it's the same for the left and right wheels):
		double wheel_radius_{};

		swerveVar::driveModel model_;

		swerveVar::ratios driveRatios_;

		swerveVar::encoderUnits units_;

		// Feed forward terms
		double f_s_;
		double f_a_;
		double f_v_;
		double f_s_v_;
		double f_s_s_;

		/// Timeout to consider cmd_vel commands old:
		double cmd_vel_timeout_{0.5};

		/// Whether to allow multiple publishers on cmd_vel topic or not:
		bool allow_multiple_cmd_vel_publishers_{true};

		/// Whether to publish odometry to tf or not:
		//bool enable_odom_tf_;

		/// Number of wheel joints:
		size_t wheel_joints_size_{0};

		/// Publish limited velocity:
		bool publish_cmd_{true};

		/**
		 * \brief Brakes the wheels, i.e. sets the velocity to 0
		 * RG: also sets to parking config
		 */
		void brake(const std::array<double, WHEELCOUNT> &steer_angles);

		/**
		 * \brief Velocity command callback
		 * \param command Velocity command message (twist)
		 */
		void cmdVelCallback(const geometry_msgs::Twist &command);
		bool changeCenterOfRotationService(talon_swerve_drive_controller::SetXY::Request &req, talon_swerve_drive_controller::SetXY::Response &res);
		bool brakeService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
		bool dontSetAngleModeService(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
		bool percentOutDriveModeService(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
		bool resetOdomService(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);

		/**
		 * \brief Get the wheel names from a wheel param
		 * \param [in]  controller_nh Controller node handler
		 * \param [in]  wheel_param   Param name
		 * \param [out] wheel_names   Vector with the whel names
		 * \return true if the wheel_param is available and the wheel_names are
		 *        retrieved successfully from the param server; false otherwise
		 */
		bool getWheelNames(ros::NodeHandle &controller_nh,
						   const std::string &wheel_param,
						   std::array<std::string, WHEELCOUNT> &wheel_names);

		/**
		 * \brief Sets odometry parameters from the URDF, i.e. the wheel radius and separation
		 * \param root_nh Root node handle
		 * \param left_wheel_name Name of the left wheel joint
		 * \param right_wheel_name Name of the right wheel joint
		 */
		/*

		bool setOdomParamsFromUrdf(ros::NodeHandle& root_nh,
		                            const std::string& steering_name,
		                            const std::string& speed_name,
		                            bool lookup_wheel_radius);

		 */
		static constexpr double DEF_ODOM_PUB_FREQ{100.};
		static const std::string DEF_ODOM_FRAME;
		static const std::string DEF_BASE_FRAME;
		static constexpr double DEF_INIT_X{0};
		static constexpr double DEF_INIT_Y{0};
		static constexpr double DEF_INIT_YAW{0};
		static constexpr double DEF_SD{0.01};

		std::array<Eigen::Vector2d, WHEELCOUNT> wheel_coords_;

		bool pub_odom_to_base_{false};       // Publish the odometry to base frame transform
		ros::Duration odom_pub_period_;    // Odometry publishing period
		Eigen::Affine2d init_odom_to_base_;  // Initial odometry to base frame transform
		Eigen::Affine2d odom_to_base_;       // Odometry to base frame transform
		Eigen::Affine2d odom_rigid_transf_;

		realtime_tools::RealtimePublisher<nav_msgs::Odometry> odom_pub_;
		tf2_ros::TransformBroadcaster odom_tf_;
		realtime_tools::RealtimePublisher<tf::tfMessage> odom_tf_pub_;
		ros::Time last_odom_pub_time_;
		ros::Time last_odom_tf_pub_time_;

		// Attempt to limit speed of wheels which are pointing further from
		// their target angle. Should help to reduce the robot being pulled
		// off course coming out of parking config or when making quick direction changes?
		bool use_cos_scaling_{false};
};


} // namespace talon_swerve_drive_controller
#endif
