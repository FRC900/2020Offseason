#pragma once

#include "ros/ros.h"
#include "frc_msgs/JoystickState.h"
#include "frc_msgs/MatchSpecificData.h"
#include "elevator_controller/CubeState.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Twist.h"
#include <string>
#include <cmath>
#include <math.h>
#include <Eigen/Dense>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <swerve_point_generator/Coefs.h>
#include <swerve_point_generator/FullGenCoefs.h>
#include <swerve_point_generator/GenerateTrajectory.h>
#include <talon_swerve_drive_controller/MotionProfilePoints.h>
#include <base_trajectory/GenerateSpline.h>
#include <talon_state_controller/TalonState.h>
#include <talon_state_controller/CustomProfileStatus.h>

void rumbleTypeConverterPublish(uint16_t leftRumble, uint16_t rightRumble);
void imuCallback(const sensor_msgs::Imu &imuState);
void cubeCallback(const elevator_controller::CubeState &cube);
void jointStateCallback(const sensor_msgs::JointState &joint_state);
void talonStateCallback(const talon_state_controller::TalonState &talon_state);

