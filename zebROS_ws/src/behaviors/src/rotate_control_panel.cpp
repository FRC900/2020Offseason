#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include "behavior_actions/RotatePanelAction.h"
#include "behavior_actions/rotate_panel_override.h"
#include "sensor_msgs/JointState.h"

class RotatePanelAction {

	protected:


		ros::NodeHandle nh_;
		actionlib::SimpleActionServer<behavior_actions::RotatePanelAction> as_; //create the actionlib server
		std::string action_name_;
		ros::ServiceClient controller_client_; //create a ros client to send requests to the controller
		behavior_actions::RotatePanelFeedback feedback_;
		behavior_actions::RotatePanelResult result_;

		//create subscribers to get data
		ros::Subscriber joint_states_sub_;

	public:


		bool preempted;
		bool timed_out;

		//make the executeCB function run every time the actionlib server is called
		RotatePanelAction(const std::string &name) :
			as_(nh_, name, boost::bind(&RotatePanelAction::executeCB, this, _1), false),
			action_name_(name)

	{

		as_.start(); //start the actionlib server

		//do networking stuff?
		std::map<std::string, std::string> service_connection_header;
		service_connection_header["tcp_nodelay"] = "1";

		//initialize the client being used to call the controller
		//ASK ADAM controller_client_ = nh_.serviceClient<control_panel_controller::ControlPanelSrv>("/frcrobot/controllers_2020/control_panel_controller", false, service_connection_header);

		//start subscribers subscribing
		//ASK ADAM joint_states_sub_ = nh_.subscribe("/frcrobot_jetson/joint_states", 1, &RotatePanelAction::jointStateCallback, this);

	}

		~RotatePanelAction(void)
		{
		}

		//define the function to be executed when the actionlib server is called
		void executeCB(const behavior_actions::RotatePanelGoalConstPtr &goal) {
			ros::Rate r(10);
			//define variables that will be re-used for each call to a controller
			double start_time;
			bool success; //if controller call succeeded
			result_.success = true;
			//define variables that will be set true if the actionlib action is to be ended
			//this will cause subsequent controller calls to be skipped, if the template below is copy-pasted
			//if both of these are false, we assume the action succeeded
			preempted = false;
			timed_out = false;

			//ROS_INFO("%s: Executing, rotating panel %i times.", action_name_.c_str(), goal->rotations);

			//send something to a controller (cargo intake in this case), copy-paste to send something else to a controller ---------------------------------------
			if(!preempted && !timed_out)
			{
				ROS_INFO("Rotate Panel Server: Rotating Panel");

				//reset variables
				start_time = ros::Time::now().toSec();
				success = false;

				//define request to send to cargo intake controller
				//HELP rotate_panel_controller::RotatePanelSrv srv;
				//HELP srv.request.rotations = goal->rotations;

				//send request to controller
				/* ASK ADAM if(!controller_client_.call(srv)) //note: the call won't happen if preempted was true, because of how && operator works
				   {
				   ROS_ERROR("Srv intake call failed in auto interpreter server intake");
				   }*/
				//update everything by doing spinny stuff

				ros::spinOnce();

				//run a loop to wait for the controller to do its work. Stop if the action succeeded, if it timed out, or if the action was preempted
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
			//end of code for sending something to a controller. Other controller calls and as servers go here


			//log state of action and set result of action
			if(timed_out)
			{
				ROS_INFO("%s: Timed Out", action_name_.c_str());
			}
			else if(preempted)
			{
				ROS_INFO("%s: Preempted", action_name_.c_str());
			}
			else //implies succeeded
			{
				ROS_INFO("%s: Succeeded", action_name_.c_str());
			}

			result_.timed_out = timed_out; //timed_out refers to last controller call, but applies for whole action
			result_.success = success; //success refers to last controller call, but applies for whole action
			as_.setSucceeded(result_); //pretend it succeeded no matter what, but tell what actually happened with the result - helps with SMACH
			return;
		}

		bool DS_Override(behavior_actions::rotate_panel_override::Request &req,
				behavior_actions::rotate_panel_override::Response &res)
		{
			if (req.override == true)
			{
				preempted = true;
				res.success = true;
				res.override = true;
				return true;
			}

			else
			{
				return false;
			}
		}
};
int main(int argc, char** argv) {
	ros::init(argc, argv, "rotate_panel_server");
	RotatePanelAction rotate_panel_action("rotate_panel_server");
	ros::NodeHandle n;
	ros::NodeHandle n_params(n, "teleop_params");
	ros::NodeHandle n_params(n, "rotateHatchPanelParams");
	if(!n_params.getParam("rotations", rotations))
        ROS_ERROR_STREAM("Could not read rotations in rotateHatchPanel");
	if(!n_params.getParam("timeout", timeout))
        ROS_ERROR_STREAM("Could not read timeout in rotateHatchPanel");
	ros::spin();
	return 0;
}
