#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include "behavior_actions/IntakeAction.h"
#include "controllers_2020_msgs/IntakeArmSrv.h"
#include "controllers_2020_msgs/IntakeRollerSrv.h"
#include "sensor_msgs/JointState.h"
#include <atomic>


//define global variables that will be defined based on config values

// TODO - these need defaults
double roller_power;
double intake_timeout;
int linebreak_debounce_iterations;
double wait_for_server_timeout;

class PowerCellIntakeAction {
	protected:
		ros::NodeHandle nh_;

		actionlib::SimpleActionServer<behavior_actions::IntakeAction> as_; //create the actionlib server
		std::string action_name_;

		//create ros clients to send requests to the controller
		ros::ServiceClient intake_arm_controller_client_;
		ros::ServiceClient intake_roller_controller_client_;

		//create actionlib clients
		actionlib::SimpleActionClient<behavior_actions::IndexerAction> ac_indexer_;

		//intake linebreak (true if ball in the intake)
		Linebreak intake_linebreak_{"intake_linebreak"};

		//define variables that will be set true if the actionlib action is to be ended
		//this will cause subsequent controller calls to be skipped
		bool preempted_ = false;
		bool timed_out_ = false;
		double start_time_;

	public:
		//make the executeCB function run every time the actionlib server is called
		PowerCellIntakeAction(const std::string &name) :
			as_(nh_, name, boost::bind(&PowerCellIntakeAction::executeCB, this, _1), false),
			action_name_(name) //TODO make sure this is linked up correctly
	{
		as_.start(); //start the actionlib server

		//do networking stuff?
		std::map<std::string, std::string> service_connection_header;
		service_connection_header["tcp_nodelay"] = "1";

		//initialize the client being used to call the controller
		intake_arm_controller_client_ = nh_.serviceClient<controllers_2020_msgs::IntakeArmSrv>("/frcrobot_jetson/intake_controller/intake_arm_command", false, service_connection_header);
		intake_roller_controller_client_ = nh_.serviceClient<controllers_2020_msgs::IntakeRollerSrv>("/frcrobot_jetson/intake_controller/intake_roller_command", false, service_connection_header);
	}

		~PowerCellIntakeAction(void)
		{
		}

		//define the function to be executed when the actionlib server is called
		void executeCB(const behavior_actions::IntakeGoalConstPtr &/*goal*/)
		{
			ROS_INFO("%s: Running callback", action_name_.c_str());

			//wait for all actionlib servers we need
			//TODO wait for indexer actionlib server

			//wait for all controller services we need
			if(! intake_arm_controller_client_.waitForExistence(ros::Duration(wait_for_server_timeout)))
			{
				ROS_ERROR_STREAM("Intake powercell server can't find powercell intake controller's arm ROS server");
				as_.setPreempted();
				return;
			}
			if(! intake_roller_controller_client_.waitForExistence(ros::Duration(wait_for_server_timeout)))
			{
				ROS_ERROR_STREAM("Intake powercell server can't find powercell intake controller's roller ROS server");
				as_.setPreempted();
				return;
			}

			//define variables that will be reused for each controller call/actionlib server call
			ros::Rate r(10);

			//define variables that will be set true if the actionlib action is to be ended
			//this will cause subsequent controller calls to be skipped, if the template below is copy-pasted
			//if both of these are false, we assume the action succeeded
			bool preempted = false;
			bool timed_out = false;
			linebreak_true_count_ = 0; //when this gets higher than linebreak_debounce_iterations, we'll consider the gamepiece intooket

			//tell indexer actionlib server to move to position intake
			//TODO



			//send command to lower arm and run roller to the powercell intake controller ------
			ROS_WARN("%s: lowering arm and spinning roller in",action_name_.c_str());
			//define requests to send to powercell intake controller
			controllers_2020_msgs::IntakeArmSrv arm_srv;
			arm_srv.request.intake_arm_extend = true;

			controllers_2020_msgs::IntakeRollerSrv roller_srv;
			roller_srv.request.percent_out = roller_power;

			//send requests to controller
			if(!intake_arm_controller_client_.call(arm_srv))
			{
				ROS_ERROR("%s: powercell intake controller call to arm failed when starting the intake", action_name_.c_str());
				preempted = true;
			}
			if(!intake_roller_controller_client_.call(roller_srv))
			{
				ROS_ERROR("%s: powercell intake controller call to roller failed when starting the intake", action_name_.c_str());
				preempted = true;
			}

			//run a loop to wait for the controller to do its work. Stop if the action succeeded, if it timed out, or if the action was preempted
			bool success = false;
			const double start_time = ros::Time::now().toSec();
			while(!success && !timed_out && !preempted && ros::ok())
			{
				if(intake_linebreak_.triggered_)
				{
					//call indexer actionlib server to intake the ball
					//sending a new call to the indexer actionlib will cancel it if it is already running
					behavior_actions::IndexerGoal indexer_goal;
					indexer_goal.action = INTAKE_ONE_BALL;
					ac_indexer_.sendGoal(indexer_goal);
					waitForActionlibServer(ac_indexer_, 10, "waiting for indexer actionlib server to intake a ball");
					//if it finishes successfully, this while loop will keep running, allowing multiple balls to be processed
				}

				if(as_.isPreemptRequested() || !ros::ok()) {
					ROS_WARN(" %s: Preempted", action_name_.c_str());
					preempted = true;
				}
				else if(!success)
				{
					r.sleep();
				}
			}

			//set ending state of controller no matter what happened: arm up and roller motors stopped
			arm_srv.request.intake_arm_extend = false;
			roller_srv.request.percent_out = 0;
			if(!intake_arm_controller_client_.call(arm_srv))
			{
				ROS_ERROR("%s: powercell intake controller call to arm failed when setting final state", action_name_.c_str());
			}
			if(!intake_roller_controller_client_.call(roller_srv))
			{
				ROS_ERROR("%s: powercell intake controller call to roller failed when setting final state", action_name_.c_str());
			}

			//log state of action and set result of action
			behavior_actions::IntakeResult result; //variable to store result of the actionlib action
			result.timed_out = timed_out; //timed_out refers to last controller call, but applies for whole action
			if(timed_out)
			{
				ROS_WARN("%s: Timed Out", action_name_.c_str());
				result.success = true;
				as_.setSucceeded(result);
			}
			else if(preempted)
			{
				ROS_WARN("%s: Preempted", action_name_.c_str());
				result.success = false;
				as_.setPreempted(result);
			}
			else //implies succeeded
			{
				ROS_WARN("%s: Succeeded", action_name_.c_str());
				result.success = true;
				as_.setSucceeded(result);
			}

			return;
		}


		void waitForActionlibServer(auto &action_client, double timeout, const std::string &activity)
			//activity is a description of what we're waiting for, e.g. "waiting for mechanism to extend" - helps identify where in the server this was called (for error msgs)
		{
			double request_time = ros::Time::now().toSec();
			ros::Rate r(10);

			//wait for actionlib server to finish
			std::string state;
			while(!preempted_ && !timed_out_ && ros::ok())
			{
				bool linebreak_true = (joint_state.position[linebreak_idx] != 0);
				if(linebreak_true)
				{
					linebreak_true_count_ += 1;
				}
				else
				{
					linebreak_true_count_ = 0;
				}
			}
			else
			{
				ROS_WARN_THROTTLE(2.0, "intake_powercell_server : intake line break sensor not found in joint_states");
				linebreak_true_count_ = 0;
			}
		}
};

int main(int argc, char** argv) {
	//create node
	ros::init(argc, argv, "powercell_intake_server");

	//create the powercell intake actionlib server
	PowerCellIntakeAction powercell_intake_action("powercell_intake_server");

	//get config values
	ros::NodeHandle n;
	ros::NodeHandle n_params_intake(n, "actionlib_powercell_intake_params");

	if (!n.getParam("/teleop/teleop_params/linebreak_debounce_iterations", linebreak_debounce_iterations))
		ROS_ERROR("Could not read linebreak_debounce_iterations in intake_server");

	if (!n.getParam("/actionlib_params/wait_for_server_timeout", wait_for_server_timeout))
		ROS_ERROR("Could not read wait_for_server_timeout in intake_sever");

	if (!n_params_intake.getParam("roller_power", roller_power))
		ROS_ERROR("Could not read roller_power in powercell_intake_server");
	if (!n_params_intake.getParam("intake_timeout", intake_timeout))
		ROS_ERROR("Could not read intake_timeout in powercell_intake_server");

	ros::AsyncSpinner Spinner(2);
	Spinner.start();
	ros::waitForShutdown();
	return 0;
}
