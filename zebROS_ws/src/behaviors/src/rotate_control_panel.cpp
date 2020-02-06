#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include "behavior_actions/RotatePanelAction.h"
#include "behavior_actions/rotate_panel_override.h"
#include "controllers_2020_msgs/ControlPanelSrv.h"


class RotatePanelAction {

	protected:


		ros::NodeHandle nh_;
		actionlib::SimpleActionServer<behavior_actions::RotatePanelAction> as_;
		std::string action_name_;
		ros::ServiceClient controller_client_;

		behavior_actions::RotatePanelFeedback feedback_;
		behavior_actions::RotatePanelResult result_;

		//nh_.getParam("rotations", rotations);
		//nh_.getParam("timeout", timeout);

		ros::Subscriber joint_states_sub_;

	public:


		bool preempted;
		bool timed_out;

	RotatePanelAction(const std::string &name) :
			as_(nh_, name, boost::bind(&RotatePanelAction::executeCB, this, _1), false),
			action_name_(name)
	{

		as_.start();

		std::map<std::string, std::string> service_connection_header;
		service_connection_header["tcp_nodelay"] = "1";

		controller_client_ = nh_.serviceClient<controllers_2020_msgs::ControlPanelSrv>("/frcrobot_jetson/control_panel_command", false, service_connection_header);

		joint_states_sub_ = nh_.subscribe("/frcrobot_jetson/talon_states", 1, &RotatePanelAction::talonStateCallback, this);
	}

		~RotatePanelAction(void)
		{
		}

		void executeCB(const behavior_actions::RotatePanelGoalConstPtr &goal)
			if(!control_panel_controller_client_.waitForExistence(ros::Duration(wait_for_server_timeout)))
			{
				ROS_ERROR_STREAM("Rotate Panel Server can't find control_panel_controller");
				as_.setPreempted();
				return 0;
			}
		void talonStateCallback()
		{
			int i = 1;
		}

		ros::Rate r(100);

		double start_time;
		bool success;

		controllers_2020_msgs::ControlPanelSrv srv;

		result_.success = true;
		preempted = false;
		timed_out = false;

		if(!preempted && !timed_out)
		{
			ROS_INFO("Rotate Panel Server: Rotating Panel");

			start_time = ros::Time::now().toSec();
			success = false;

			srv.request.control_panel_rotations = 3.3; //add env variable stuff
			srv.request.panel_arm_extend = true;

			if(!controller_client_.call(srv))
			{
				ROS_ERROR("Srv intake call failed in auto interpreter server intake");
				preempted = true;
			}

			while(!success && !timed_out && !preempted) {
				if(as_.isPreemptRequested() || !ros::ok()) {
					ROS_WARN("%s: Preempted", action_name_.c_str());
					as_.setPreempted();
					preempted = true;
				}
				if (!preempted) {
					r.sleep();
					ros::spinOnce();
					timed_out = (ros::Time::now().toSec()-start_time) > timeout;
				}
			}
		}

		if(timed_out)
		{
			ROS_INFO("%s: Timed Out", action_name_.c_str());
		}
		else if(preempted)
		{
			ROS_INFO("%s: Preempted", action_name_.c_str());
		}
		else
		{
			ROS_INFO("%s: Succeeded", action_name_.c_str());
		}

		result_.timed_out = timed_out;
		result_.success = success;
		as_.setSucceeded(result_);
		return true;
};

bool DS_Override(behavior_actions::rotate_panel_override::Request &req,
		behavior_actions::rotate_panel_override::Response &res)
{
	if (req.override == true)
	{
		//RotatePanelAction.preempted = true;
		res.success = true;
		res.override = true;
		return true;
	}

	else
	{
		return false;
	}
}
int main(int argc, char** argv) {
	ros::init(argc, argv, "rotate_panel_server");
	RotatePanelAction rotate_panel_action("rotate_panel_server");
	ros::NodeHandle n;
	ros::NodeHandle n_params(n, "teleop_params");
	ros::NodeHandle n_rotate_params(n, "rotateHatchPanelParams");
	ros::spin();
	return 0;
}
