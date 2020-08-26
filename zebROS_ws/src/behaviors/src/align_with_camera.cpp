// Node which reads camera data, translates coordinates to frame of the
// mechanism which needs to be lined up, and publishes a distance needed
// to move to line up with the camera detection
// TODO : debate splitting this up - it kinda combines a few things
// which might be better separate - translating between frames, adding latency
// compensation and then adding a controller to move towards the target
#include <ros/ros.h>
#include <cmath>
#include <vector>
#include "geometry_msgs/PointStamped.h"
#include "message_filters/subscriber.h"
#include "std_srvs/SetBool.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
//tf stuff
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/message_filter.h"

bool publish = false;
//bool goals_found = false;
tf2_ros::Buffer buffer;
std::string target_frame;
geometry_msgs::PointStamped relative_goal_location;
ros::Time goal_timestamp;

constexpr bool debug = true;

void cameraCB(const geometry_msgs::PointStampedConstPtr& raw_goal_location)
{
	//goals_found = true;
	try
	{
		if(debug)
		{
			ROS_INFO_THROTTLE(1, " RAW point of turtle 3 in frame of turtle 1 Position(x:%f y:%f z:%f)\n",
					raw_goal_location->point.x,
					raw_goal_location->point.y,
					raw_goal_location->point.z);
		}
		buffer.transform(*raw_goal_location, relative_goal_location, target_frame);
		goal_timestamp = raw_goal_location->header.stamp;
		if(debug)
		{
			ROS_INFO_THROTTLE(1, "RELATIVE point of turtle 3 in frame of turtle 1 Position(x:%f y:%f z:%f)\n",
					relative_goal_location.point.x,
					relative_goal_location.point.y,
					relative_goal_location.point.z);
		}
	}
	catch (tf2::TransformException &ex)
	{
		ROS_WARN("Failed %s\n", ex.what());
	}
}

bool startStopAlign(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
	if (debug)
		ROS_INFO_STREAM("called align_with_camera startStopAlign(" << req.data << ")");
	publish = req.data;
	res.success = true;

	return true;
}

void startStopCallback(std_msgs::Bool msg)
{
	if (debug)
		ROS_INFO_STREAM("called align_with_camera startStopCallback(" << msg.data << ")");
	publish = msg.data;
}

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "align_with_camera");
	ros::NodeHandle n;
	ros::NodeHandle n_params(n, "align_with_camera_params");
	ros::NodeHandle n_private_params("~");

	double last_command_published = 0.0;

	//read configs
	double cmd_vel_to_pub;
	if(!n_params.getParam("cmd_vel_to_pub", cmd_vel_to_pub))
		ROS_ERROR_STREAM("Could not read cmd_vel_to_pub in align_with_camera");
	double error_threshold;
	if(!n_params.getParam("error_threshold", error_threshold))
		ROS_ERROR_STREAM("Could not read error_threshold in align_with_camera");
	if(!n_private_params.getParam("target_frame", target_frame))
		ROS_ERROR_STREAM("Could not read target_frame in align_with_camera");
	std::string axis;
	if(!n_private_params.getParam("axis", axis))
		ROS_ERROR_STREAM("Could not read axis in align_with_camera");
	std::string state_publisher_topic;
	if(!n_private_params.getParam("state_publisher_topic", state_publisher_topic))
		ROS_ERROR_STREAM("Could not read state_publisher_topic in align_with_camera");
	std::string enable_subscriber_topic;
	if(!n_private_params.getParam("enable_subscriber_topic", enable_subscriber_topic))
		ROS_ERROR_STREAM("Could not read enable_subscriber_topic in align_with_camera");
	std::string setpoint_publisher_topic;
	if(!n_private_params.getParam("setpoint_publisher_topic", setpoint_publisher_topic))
		ROS_ERROR_STREAM("Could not read setpoint_publisher_topic in align_with_camera");
	if((target_frame != "panel_outtake") && (target_frame != "cargo_outtake"))
	{
		ROS_ERROR("Unknown target_frame in align_with_camera");
		return -1;
	}

	double extra_latency = 0.0;
	if(!n_private_params.getParam("extra_latency", extra_latency))
		ROS_ERROR_STREAM("Could not read extra_latency in align_with_camera");

	//set up publisher for publish_pid_cmd_vel node
	ros::Publisher command_pub = n.advertise<std_msgs::Float64>("align_with_camera/command", 1);
	//set up feedback publisher for the align_server which uses this node
	ros::Publisher successful_y_align = n.advertise<std_msgs::Float64MultiArray>(state_publisher_topic, 1);
	//set up enable subscriber from align_server
	ros::Subscriber start_stop_sub = n.subscribe(enable_subscriber_topic, 1, &startStopCallback);
	//set up camera subscriber with transforms
	message_filters::Subscriber<geometry_msgs::PointStamped> camera_msg_sub(n, "pointstamped_goal_msg", 1);
	//advertise service to start or stop align (who uses this?)
	ros::ServiceServer start_stop_service = n.advertiseService("align_with_camera", startStopAlign);
	//set up setpoint publisher
	ros::Publisher setpoint_pub = n.advertise<std_msgs::Float64>(setpoint_publisher_topic, 1);

	//set up transforms for camera -> mechanism
	tf2_ros::TransformListener tf2(buffer);
	tf2_ros::MessageFilter<geometry_msgs::PointStamped> tf2_filter(camera_msg_sub, buffer, target_frame, 10, 0);
	tf2_filter.registerCallback(cameraCB);

	std_msgs::Float64 cmd_msg;
	cmd_msg.data = 0;

	std_msgs::Float64MultiArray aligned_msg;
	aligned_msg.data.resize(5);

	ros::Rate r(60);

	bool publish_last = false;
	while(ros::ok())
	{
		//bool aligned = false;
		double error;
		if(axis == "y")
		{
			error = relative_goal_location.point.y;
		}
		else if (axis == "x")
		{
			error = relative_goal_location.point.x;
		}
		else
		{
			ROS_ERROR_STREAM("AXIS IS NOT X OR Y IN ALIGN_WITH_CAMERA");
			break;
		}

		// Very basic latency compensation - assumes we've been moving a constant speed
		// since the last camera frame was published.  For a more accurate estimate, keep
		// a vector of <timestamp, velocity> pairs and work backwards from now until the
		// goal timestamp. For each entry, sum up (time between prev and curr entry) * entry velocity
		// then interpolate the distance moved for the saved entry where the code jumps past
		// the goal timestamp.
		//const double latency_comp = last_command_published * ((ros::Time::now() - goal_timestamp).toSec() + extra_latency);
		//if(debug)
		//	ROS_INFO_STREAM_THROTTLE(1, "Latency_comp = " << latency_comp << " error before = " << error);
		//error += latency_comp;

		if(fabs(error) < error_threshold)
		{
			if(debug)
				ROS_INFO_STREAM_THROTTLE(1, "we're aligned!! error = " << error);
			cmd_msg.data = 0;
		}
		else if(error > 0)
		{
			if(debug)
				ROS_INFO_STREAM_THROTTLE(1, "we're left. error = " << error);
			cmd_msg.data = -1*cmd_vel_to_pub;
		}
		else
		{
			if(debug)
				ROS_INFO_STREAM_THROTTLE(1, "we're right. error = " << error);
			cmd_msg.data = 1*cmd_vel_to_pub;
		}

		if(publish)
		{
			command_pub.publish(cmd_msg);
		}
		else if(!publish && publish_last)
		{
			cmd_msg.data = 0;
			command_pub.publish(cmd_msg);
		}
		last_command_published = cmd_msg.data;

		// Mimic PID node debug info - index 0 is error,
		// which is all that used by the align server code
		aligned_msg.data[0] = error;
		successful_y_align.publish(aligned_msg);
		std_msgs::Float64 msg;
		msg.data = 0.0;
		setpoint_pub.publish(msg);

		publish_last = publish;
		ros::spinOnce();
		r.sleep();
	}

	return 0;
}
