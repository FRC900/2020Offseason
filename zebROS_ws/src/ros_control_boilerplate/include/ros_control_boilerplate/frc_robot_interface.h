/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, University of Colorado, Boulder
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
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
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

/* Original Author: Dave Coleman
   Desc:   Example ros_control hardware interface that performs a perfect control loop for
   simulation
*/

#ifndef FRC_ROBOT_INTERFACE_INC_
#define FRC_ROBOT_INTERFACE_INC_

#include <atomic>
#include <thread>

// ROS
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_mode_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>
#include <realtime_tools/realtime_publisher.h>
#include <urdf/model.h>

// ROS Controls
#include "as726x_interface/as726x_interface.h"
#include "frc_interfaces/joystick_interface.h"
#include "frc_interfaces/match_data_interface.h"
#include "frc_interfaces/pcm_state_interface.h"
#include "frc_interfaces/pdp_state_interface.h"
#include "frc_interfaces/robot_controller_interface.h"
#include "remote_joint_interface/remote_joint_interface.h"
#include "ros_control_boilerplate/ros_iterative_robot.h"
#include "ros_control_boilerplate/talon_convert.h"
#include "talon_interface/cancoder_command_interface.h"
#include "talon_interface/canifier_command_interface.h"
#include "talon_interface/orchestra_command_interface.h"
#include "talon_interface/talon_command_interface.h"

#include "ros_control_boilerplate/tracer.h"

// WPILIB stuff
#include <hal/FRCUsageReporting.h>
#include <hal/HALBase.h>
#include <hal/Types.h>

// CTRE
#include <ctre/phoenix/motorcontrol/IMotorController.h>

// Use forward declarations to avoid including a whole bunch of
// WPIlib headers we don't care about - this speeds up the build process
class AHRS;
namespace frc { class AnalogInput; }
namespace frc { class DigitalInput; }
namespace frc { class DigitalOutput; }
namespace frc { class Joystick; }
namespace frc { class NidecBrushless; }
namespace frc { class PWM; }

namespace ros_control_boilerplate
{

// Joint used to communicate internally in the hw
// interface(s) by name rather than via some index
// in an array. Typically used for communicating with
// the DS in some way
class DummyJoint
{
	public :
		DummyJoint(const std::string &name, double *address) :
			name_(name), address_(address)
		{
			name_.erase(name.find_last_not_of('_') + 1);
		}
		std::string name_;
		double *address_;
};
#define Dumify(name) ros_control_boilerplate::DummyJoint(#name, &name)

class CustomProfileState
{
	public:
		CustomProfileState()
			: time_start_(ros::Time::now().toSec())
			, iteration_count_(0)
			, points_run_(0)
	{
	}

	double time_start_;
	int iteration_count_;
	int points_run_;
	hardware_interface::CustomProfileStatus status_;
	std::vector<std::vector<hardware_interface::CustomProfilePoint>> saved_points_;
	std::vector<std::vector<double>> saved_times_;
};

//Stuff from frcrobot_hw_interface
class DoubleSolenoidHandle
{
	public:
		DoubleSolenoidHandle(HAL_SolenoidHandle forward, HAL_SolenoidHandle reverse)
			: forward_(forward)
			  , reverse_(reverse)
		{
		}
		HAL_SolenoidHandle forward_;
		HAL_SolenoidHandle reverse_;
};

/// \brief Hardware interface for a robot
class FRCRobotInterface : public hardware_interface::RobotHW
{
	public:
		//******Stuff from frcrobot_hw_interface
		/**
		 * \brief Constructor
		 * \param nh - Node handle for topics.
		 * \param urdf - optional pointer to a parsed robot model
		 */
		FRCRobotInterface(ros::NodeHandle &nh, urdf::Model *urdf_model = NULL);
		~FRCRobotInterface();

		/** \brief Initialize the hardware interface */
		virtual bool init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh) override;

		/** \brief Read the state from the robot hardware. */
		virtual void read(const ros::Time& time, const ros::Duration& period) override = 0;

		/** \brief Write the command to the robot hardware. */
		virtual void write(const ros::Time& time, const ros::Duration& period) override = 0;

		/** \brief Set all members to default values */
		virtual void reset();

		//******
		/**
		 * \brief Check (in non-realtime) if given controllers could be started and stopped from the
		 * current state of the RobotHW
		 * with regard to necessary hardware interface switches. Start and stop list are disjoint.
		 * This is just a check, the actual switch is done in doSwitch()
		 */
		virtual bool prepareSwitch(const std::list<hardware_interface::ControllerInfo> &/*start_list*/,
							       const std::list<hardware_interface::ControllerInfo> &/*stop_list*/) override;

		/**
		 * \brief Perform (in non-realtime) all necessary hardware interface switches in order to start
		 * and stop the given controllers.
		 * Start and stop list are disjoint. The feasability was checked in canSwitch() beforehand.
		 */
		virtual void doSwitch(const std::list<hardware_interface::ControllerInfo> &/*start_list*/,
							  const std::list<hardware_interface::ControllerInfo> &/*stop_list*/) override
		{
		}

		/** \brief Helper for debugging a joint's state */
		virtual void printState();
		std::string printStateHelper();

		/** \brief Helper for debugging a joint's command */
		std::string printCommandHelper();

	protected:
		/** \brief Get the URDF XML from the parameter server */
		virtual void loadURDF(ros::NodeHandle &nh, std::string param_name);
		virtual std::vector<DummyJoint> getDummyJoints(void) { return std::vector<DummyJoint>();}

		// Short name of this class
		std::string name_;

		// Hardware interfaces
		hardware_interface::JointStateInterface                joint_state_interface_;
		hardware_interface::TalonStateInterface                talon_state_interface_;
		hardware_interface::RemoteTalonStateInterface          talon_remote_state_interface_;
		hardware_interface::canifier::CANifierStateInterface   canifier_state_interface_;
		hardware_interface::canifier::RemoteCANifierStateInterface canifier_remote_state_interface_;
		hardware_interface::cancoder::CANCoderStateInterface   cancoder_state_interface_;
		hardware_interface::cancoder::RemoteCANCoderStateInterface cancoder_remote_state_interface_;
		hardware_interface::PDPStateInterface	               pdp_state_interface_;
		hardware_interface::RemotePDPStateInterface	           pdp_remote_state_interface_;
		hardware_interface::PCMStateInterface	               pcm_state_interface_;
		hardware_interface::RemotePCMStateInterface	           pcm_remote_state_interface_;
		hardware_interface::JoystickStateInterface             joystick_state_interface_;
		hardware_interface::MatchStateInterface                match_state_interface_;
		hardware_interface::RemoteMatchStateInterface          match_remote_state_interface_;
		hardware_interface::as726x::AS726xStateInterface       as726x_state_interface_;
		hardware_interface::as726x::RemoteAS726xStateInterface as726x_remote_state_interface_;
                hardware_interface::OrchestraStateInterface            talon_orchestra_state_interface_;

		hardware_interface::JointCommandInterface          joint_command_interface_;
		hardware_interface::PositionJointInterface         joint_position_interface_;
		hardware_interface::VelocityJointInterface         joint_velocity_interface_;
		hardware_interface::EffortJointInterface           joint_effort_interface_;
		hardware_interface::RemoteJointInterface           joint_remote_interface_;
		hardware_interface::TalonCommandInterface          talon_command_interface_;
		hardware_interface::canifier::CANifierCommandInterface canifier_command_interface_;
		hardware_interface::cancoder::CANCoderCommandInterface cancoder_command_interface_;
		hardware_interface::as726x::AS726xCommandInterface as726x_command_interface_;
		hardware_interface::ImuSensorInterface             imu_interface_;
		hardware_interface::RemoteImuSensorInterface       imu_remote_interface_;
                hardware_interface::OrchestraCommandInterface      talon_orchestra_command_interface_;

		hardware_interface::RobotControllerStateInterface  robot_controller_state_interface_;

		hardware_interface::JointModeInterface            joint_mode_interface_;
		hardware_interface::RemoteJointModeInterface      joint_mode_remote_interface_;

		std::vector<CustomProfileState> custom_profile_state_;

		void custom_profile_write(int joint_id);
		void custom_profile_set_talon(hardware_interface::TalonMode mode, double setpoint, double fTerm, int joint_id, int pidSlot, bool zeroPos);

		void readJointLocalParams(XmlRpc::XmlRpcValue joint_params,
								  const bool local,
								  const bool saw_local_keyword,
								  bool &local_update,
								  bool &local_hardware);
		void readConfig(ros::NodeHandle rpnh);
		void createInterfaces(void);
		bool initDevices(ros::NodeHandle root_nh);

		// Configuration
		std::vector<std::string> can_ctre_mc_names_;
		std::vector<int>         can_ctre_mc_can_ids_;
		std::vector<bool>        can_ctre_mc_local_updates_;
		std::vector<bool>        can_ctre_mc_local_hardwares_;
		std::vector<bool>        can_ctre_mc_is_talon_fx_;
		std::vector<bool>        can_ctre_mc_is_talon_srx_;
		std::size_t              num_can_ctre_mcs_{0};

		std::vector<std::string> canifier_names_;
		std::vector<int>         canifier_can_ids_;
		std::vector<bool>        canifier_local_updates_;
		std::vector<bool>        canifier_local_hardwares_;
		std::size_t              num_canifiers_{0};

		std::vector<std::string> cancoder_names_;
		std::vector<int>         cancoder_can_ids_;
		std::vector<bool>        cancoder_local_updates_;
		std::vector<bool>        cancoder_local_hardwares_;
		std::size_t              num_cancoders_{0};

		std::vector<std::string> nidec_brushless_names_;
		std::vector<int>         nidec_brushless_pwm_channels_;
		std::vector<int>         nidec_brushless_dio_channels_;
		std::vector<bool>        nidec_brushless_inverts_;
		std::vector<bool>        nidec_brushless_local_updates_;
		std::vector<bool>        nidec_brushless_local_hardwares_;
		std::size_t              num_nidec_brushlesses_{0};

		//I think inverts are worth having on below 3
		std::vector<std::string> digital_input_names_;
		std::vector<int>         digital_input_dio_channels_;
		std::vector<bool>        digital_input_inverts_;
		std::vector<bool>        digital_input_locals_;
		std::size_t              num_digital_inputs_{0};

		std::vector<std::string> digital_output_names_;
		std::vector<int>         digital_output_dio_channels_;
		std::vector<bool>        digital_output_inverts_;
		std::vector<bool>        digital_output_local_updates_;
		std::vector<bool>        digital_output_local_hardwares_;
		std::size_t              num_digital_outputs_{0};

		std::vector<std::string> pwm_names_;
		std::vector<int>         pwm_pwm_channels_;
		std::vector<bool>        pwm_inverts_;
		std::vector<bool>        pwm_local_updates_;
		std::vector<bool>        pwm_local_hardwares_;
		std::size_t              num_pwms_{0};

		std::vector<std::string> solenoid_names_;
		std::vector<int>         solenoid_ids_;
		std::vector<int>         solenoid_pcms_;
		std::vector<bool>        solenoid_local_updates_;
		std::vector<bool>        solenoid_local_hardwares_;
		std::size_t              num_solenoids_{0};

		std::vector<std::string> double_solenoid_names_;
		std::vector<int>         double_solenoid_forward_ids_;
		std::vector<int>         double_solenoid_reverse_ids_;
		std::vector<int>         double_solenoid_pcms_;
		std::vector<bool>        double_solenoid_local_updates_;
		std::vector<bool>        double_solenoid_local_hardwares_;
		std::size_t              num_double_solenoids_{0};

		std::vector<std::string> compressor_names_;
		std::vector<int>         compressor_pcm_ids_;
		std::vector<bool>        compressor_local_updates_;
		std::vector<bool>        compressor_local_hardwares_;
		std::size_t              num_compressors_{0};

		std::vector<std::string> pdp_names_;
		std::vector<int32_t>     pdp_modules_;
		std::vector<bool>        pdp_locals_;
		std::size_t              num_pdps_{0};

		std::vector<std::string> rumble_names_;
		std::vector<int>         rumble_ports_;
		std::vector<bool>        rumble_local_updates_;
		std::vector<bool>        rumble_local_hardwares_;
		std::size_t              num_rumbles_{0};

		std::vector<std::string> navX_names_;
		std::vector<std::string> navX_frame_ids_;
		std::vector<int>         navX_ids_;
		std::vector<bool>        navX_locals_;
		std::size_t              num_navX_{0};

		std::vector<std::string> analog_input_names_;
		std::vector<int>         analog_input_analog_channels_;
		std::vector<double>      analog_input_a_;
		std::vector<double>      analog_input_b_;
		std::vector<bool>        analog_input_locals_;
		std::size_t              num_analog_inputs_{0};

		std::vector<std::string> dummy_joint_names_;
		std::vector<bool>        dummy_joint_locals_; // Not sure if this is needed?
		std::size_t              num_dummy_joints_{0};

		std::vector<std::string> ready_signal_names_;
		std::vector<bool>        ready_signal_locals_;
		std::size_t              num_ready_signals_{0};

		std::vector<std::string> joystick_names_;
		std::vector<int>         joystick_ids_; // pretty sure this is montonic increasing by default?
		std::size_t              num_joysticks_{0};

		std::vector<std::string> as726x_names_;
		std::vector<std::string> as726x_ports_;
		std::vector<int>         as726x_addresses_;
		std::vector<bool>        as726x_local_updates_;
		std::vector<bool>        as726x_local_hardwares_;
		std::size_t              num_as726xs_{0};

                std::vector<std::string> talon_orchestra_names_;
                std::size_t              num_talon_orchestras_{0};
                std::vector<int>         talon_orchestra_ids_;

		bool run_hal_robot_{true};
		std::string can_interface_{"can0"};

		urdf::Model *urdf_model_{nullptr};

		// Array holding master cached state of hardware resources
		std::vector<hardware_interface::TalonHWState> talon_state_;
		std::vector<hardware_interface::canifier::CANifierHWState> canifier_state_;
		std::vector<hardware_interface::cancoder::CANCoderHWState> cancoder_state_;
		std::vector<double> brushless_vel_;

		std::vector<double> digital_input_state_;
		std::vector<double> digital_output_state_; //No actual data
		std::vector<double> pwm_state_; //No actual data
		std::vector<double> solenoid_state_;
		std::vector<double> solenoid_pwm_state_;
		std::vector<double> double_solenoid_state_;
		std::vector<double> rumble_state_; //No actual data
		std::vector<double> compressor_state_;
		std::vector<hardware_interface::PDPHWState> pdp_state_;
		std::vector<hardware_interface::PCMState> pcm_state_;
		hardware_interface::RobotControllerState robot_controller_state_;
		std::vector<hardware_interface::JoystickState> joystick_state_;
		hardware_interface::MatchHWState match_data_;
	    std::vector<hardware_interface::OrchestraState> orchestra_state_;
		std::mutex match_data_mutex_;
		std::vector<std::shared_ptr<std::mutex>> joystick_sim_write_mutex_;

		// Each entry in the vector is an array. That array holds
		// the data returned from one particular imu
		std::vector<std::array<double,4>> imu_orientations_; // x,y,z,w
		std::vector<std::array<double,9>> imu_orientation_covariances_; // [x,y,z] x [x,y,z]
		std::vector<std::array<double,3>> imu_angular_velocities_; //x,y,z
		std::vector<std::array<double,9>> imu_angular_velocity_covariances_;
		std::vector<std::array<double,3>> imu_linear_accelerations_; // x,y,z
		std::vector<std::array<double,9>> imu_linear_acceleration_covariances_;

		std::vector<double> analog_input_state_;

		std::vector<hardware_interface::as726x::AS726xState> as726x_state_;

		// Same as above, but for pending commands to be written to the hardware
		std::vector<hardware_interface::TalonHWCommand> talon_command_;
		std::vector<hardware_interface::canifier::CANifierHWCommand> canifier_command_;
		std::vector<hardware_interface::cancoder::CANCoderHWCommand> cancoder_command_;
		std::vector<double> brushless_command_;
		std::vector<double> digital_output_command_;
		std::vector<double> pwm_command_;
		std::vector<double> solenoid_command_;
		std::vector<hardware_interface::JointCommandModes> solenoid_mode_;
		std::vector<hardware_interface::JointCommandModes> prev_solenoid_mode_;
		std::vector<double> double_solenoid_command_;
		std::vector<double> rumble_command_;
		std::vector<double> compressor_command_;
                std::vector<hardware_interface::OrchestraCommand> orchestra_command_;

		std::vector<double> dummy_joint_position_;
		std::vector<double> dummy_joint_velocity_;
		std::vector<double> dummy_joint_effort_;
		std::vector<double> dummy_joint_command_;

		std::vector<hardware_interface::as726x::AS726xCommand> as726x_command_;

		std::vector<double> robot_ready_signals_;
		bool                robot_code_ready_{false};
		bool                last_robot_enabled_{false};

		//certain data will be read at a slower rate than the main loop, for computational efficiency
		//robot iteration calls - sending stuff to driver station
		double ctre_mc_read_hz_{100};
		double pcm_read_hz_{20};
		double pdp_read_hz_{20};
		double t_prev_robot_iteration_;
		double robot_iteration_hz_{50};

		double t_prev_joystick_read_;
		double joystick_read_hz_{50};

		double t_prev_match_data_read_;
		double match_data_read_hz_{2};

		double t_prev_robot_controller_read_;
		double robot_controller_read_hz_{20};

		/* Get conversion factor for position, velocity, and closed-loop stuff */
		double getConversionFactor(int encoder_ticks_per_rotation, hardware_interface::FeedbackDevice encoder_feedback, hardware_interface::TalonMode talon_mode) const;

		// Count sequential CAN errors
		size_t can_error_count_{0};
		bool safeTalonCall(ctre::phoenix::ErrorCode error_code, const std::string &talon_method_name, const int talon_id);

		std::vector<std::shared_ptr<ctre::phoenix::motorcontrol::IMotorController>> ctre_mcs_;

		// Maintain a separate read thread for each talon SRX
		std::vector<std::shared_ptr<std::mutex>> ctre_mc_read_state_mutexes_;
		std::vector<std::shared_ptr<hardware_interface::TalonHWState>> ctre_mc_read_thread_states_;
		std::vector<std::thread> ctre_mc_read_threads_;
		std::atomic<bool> skip_bus_voltage_temperature_{false};
		void ctre_mc_read_thread(std::shared_ptr<ctre::phoenix::motorcontrol::IMotorController> ctre_mc,
												 std::shared_ptr<hardware_interface::TalonHWState> state,
												 std::shared_ptr<std::mutex> mutex,
												 std::unique_ptr<Tracer> tracer,
												 double poll_frequency);

		std::vector<std::shared_ptr<frc::NidecBrushless>> nidec_brushlesses_;
		std::vector<std::shared_ptr<frc::DigitalInput>> digital_inputs_;
		std::vector<std::shared_ptr<frc::DigitalOutput>> digital_outputs_;
		std::vector<std::shared_ptr<frc::PWM>> PWMs_;
		std::vector<HAL_SolenoidHandle> solenoids_;
		std::vector<DoubleSolenoidHandle> double_solenoids_;
		std::vector<std::shared_ptr<AHRS>> navXs_;
		std::vector<std::shared_ptr<frc::AnalogInput>> analog_inputs_;

		std::vector<std::shared_ptr<std::mutex>> pcm_read_thread_mutexes_;
		std::vector<std::shared_ptr<hardware_interface::PCMState>> pcm_read_thread_state_;
		void pcm_read_thread(HAL_CompressorHandle compressor_handle,
							 int32_t pcm_id,
							 std::shared_ptr<hardware_interface::PCMState> state,
							 std::shared_ptr<std::mutex> mutex,
							 std::unique_ptr<Tracer> tracer,
							 double poll_frequency);
		std::vector<std::thread> pcm_threads_;
		std::vector<HAL_CompressorHandle> compressors_;

		std::vector<std::shared_ptr<std::mutex>> pdp_read_thread_mutexes_;
		std::vector<std::shared_ptr<hardware_interface::PDPHWState>> pdp_read_thread_state_;
		void pdp_read_thread(int32_t pdp,
							 std::shared_ptr<hardware_interface::PDPHWState> state,
							 std::shared_ptr<std::mutex> mutex,
							 std::unique_ptr<Tracer> tracer,
							 double poll_frequency);
		std::vector<std::thread> pdp_threads_;

		std::vector<std::shared_ptr<frc::Joystick>> joysticks_;

		std::unique_ptr<ROSIterativeRobot> robot_{nullptr};
		Tracer read_tracer_;

		talon_convert::TalonConvert talon_convert_;

};  // class

}  // namespace
#endif
