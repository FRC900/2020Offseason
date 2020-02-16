#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include "actionlib/client/simple_action_client.h"
#include <atomic>
#include <ros/console.h>

//include action files - for this actionlib server and any it sends requests to
#include "behavior_actions/Climb2020Action.h"
#include "behavior_actions/AlignBarAction.h"
#include "behavior_actions/WinchAction.h"

//include controller service files and other service files
#include "controllers_2020_msgs/ClimbResulterSrv.h"
#include "sensor_msgs/JointState.h" 

//define global variables that will be defined based on config values
double climber_deploy_timeout;
double climber_climb_timeout;
double winch_pull_up_pause_time;

//create the class for the actionlib server
class ClimberAction {
	protected:
		ros::NodeHandle nh_;

		actionlib::SimpleActionServer<behavior_actions::ClimbAction> as_climber_;
		std::string action_name_;

		actionlib::SimpleActionServer<behavior_actions::WinchAction> as_winch_; //create the actionlib server
		std::string action_name_;

		//clients to call other actionlib servers
		actionlib::SimpleActionClient<behavior_actions::AlignBarAction> ac_alignBar_;

		//clients to call controllers
		ros::ServiceClient climber_controller_client_; //create a ros client to send requests to the climber controller
		
		ros::Publisher cmd_vel_publisher_;
		behavior_actions::ClimbResult result_; //variable to store result of the actionlib action

		//create subscribers to get data
		ros::Subscriber joint_states_sub_;
		ros::Subscriber talon_states_sub_;

		//variables to store if server was preempted_ or timed out. If either true, skip everything (if statements). If both false, we assume success.
		bool preempted_;
		bool timed_out_;
		ros::Rate r{10}; //used for wait loops, curly brackets needed so it doesn't think this is a function
		double start_time_;

        //config variables, with defaults
        double server_timeout_; //overall timeout for your server
        double wait_for_server_timeout_; //timeout for waiting for other actionlib servers to become available before exiting this one

	public:
		//Constructor - create actionlib server; the executeCB function will run every time the actionlib server is called
		ClimberAction(const std::string &name, server_timeout, wait_for_server_timeout) :
			as_(nh_, name, boost::bind(&ClimberAction::executeCB, this, _1), false),
			action_name_(name),
            server_timeout_(server_timeout),
            wait_for_server_timeout_(wait_for_server_timeout)
			//ac_elevator_("/elevator/elevator_server", true) example how to initialize other action clients, don't forget to add a comma on the previous line
			ac_winch_("/winch/winch_server", true);
	{
		as_.start(); //start the actionlib server

		//do networking stuff
		std::map<std::string, std::string> service_connection_header;
		service_connection_header["tcp_nodelay"] = "1";

		//initialize client used to call controllers
		//e.g. mech_controller_client_ = nh_.serviceClient<controller_package::ControllerSrv>("name_of_service", false, service_connection_header);
		climber_controller_client_ = nh_.serviceClient<controllers_2020_msgs::ClimberSrv>("/.dockerenvfrcrobot_jetson/climber_controller/climber_command", false, service_connection_header);
		
		talon_states_sub_ = nh_.subscribe("/frcrobot_jetson/talon_states", 1, &ClimbAction::talonStateCallback, this);

		cmd_vel_publisher_ = nh_.advertise<geometry_msgs::Twist>("swerve_drive_controller/cmd_vel", 1);
		joint_states_sub_ = nh_.subscribe("/frcrobot_jetson/joint_states", 1, &ClimbAction::jointStateCallback, this);
	}

		~ClimberAction(void)
		{
		}


		//Use to make pauses while still checking timed_out_ and preempted_
		bool pause(const double duration, const std::string &activity)
		{
			const double pause_start_time = ros::Time::now().toSec();

			while(!preempted_ && !timed_out_ && ros::ok())
			{
				if(as_.isPreemptRequested() || !ros::ok())
				{
					preempted_ = true;
					ROS_ERROR_STREAM("climber_server: preempt during pause() - " << activity);
				}
				else if ((ros::Time::now().toSec() - start_time_) >= server_timeout_)
				{
					timed_out_ = true;
					ROS_ERROR_STREAM("climber_server: timeout during pause() - " << activity);
				}

				if((ros::Time::now().toSec() - pause_start_time) >= duration)
				{
					return true; //pause worked like expected
				}
			}

			return false; //wait loop must've been broken by preempt, global timeout, or ros not ok
		}

		//define the function to be executed when the actionlib server is called
		void executeCB(const behaviors::ClimbGoalConstPtr &goal)
		{
			ROS_INFO("%s: Running callback", action_name_.c_str());

			start_time_ = ros::Time::now().toSec();
			preempted_ = false;
			timed_out_ = false;

			//wait for all actionlib servers we need
			if(!ac_winch_.waitForServer(ros::Duration(wait_for_server_timeout_)))
			{
				ROS_ERROR_STREAM(action_name_ << " couldn't find align_winch actionlib server");
				as_.setPreempted();
				return;
			}


			//wait for all controller servers we need
			
			if(! climber_controller_client_.waitForExistence(ros::Duration(wait_for_server_timeout_)))
			{
				ROS_ERROR_STREAM(action_name_ << " can't find climber_controller");
				as_.setPreempted();
				return;
			}
			
			if(goal->step == 0)
			{
				ROS_INFO("Running climber server step 0");

				//deploy piston
				if(!preempted && !timed_out && ros::ok())
				{
					ROS_INFO("climber server step 0: deploying piston");
					controllers_2020_msgs::ClimberSrv climber_srv;
					climber_srv.request.climber_deploy = true;
				}
			}

			//Basic controller call ---------------------------------------
			if(!preempted_ && !timed_out_ && ros::ok())
			{
				ROS_INFO("climber_server: what this is doing");
				//call controller client, if failed set preempted_ = true, and log an error msg

				//if necessary, run a loop to wait for the controller to finish
				while(!preempted_ && !timed_out_ && ros::ok())
				{
					//check preempted_
					if(as_.isPreemptRequested() || !ros::ok()) {
						ROS_ERROR_STREAM(action_name_ << ": preempt while calling ______ controlle");
						preempted_ = true;
					}
					//test if succeeded, if so, break out of the loop
					else if(test here) {
						break;
					}
					//check timed out - TODO might want to use a timeout for this specific controller call rather than the whole server's timeout?
					else if (ros::Time::now().toSec() - start_time_ > server_timeout_) {
						ROS_ERROR_STREAM(action_name_ << ": timed out while calling ______ controller");
						timed_out_ = true;
					}
					//otherwise, pause then loop again
					else {
						r.sleep();
					}
				}
			}
			//preempt handling (skip this and set final state at end if only 1 possble final state)
			/*
			if(preempted_ || timed_out_ || !ros::ok())
			{}
			*/

			//if necessary, pause a bit between doing things (between piston firings usually)
			/* e.g.
			pause(sec_to_pause, "what we're pausing for");
			*/





			//Basic actionlib server call -------------------------------------
			if(!preempted_ && !timed_out_ && ros::ok())
			{
				ROS_INFO("what this is doing");
				//Call actionlib server
				/* e.g.
				behaviors::ElevatorGoal elevator_goal;
				elevator_goal.place_cargo = true;
				ac_elevator_.sendGoal(elevator_goal);
				*/
				//wait for actionlib server
				//e.g. waitForActionlibServer(ac_elevator_, 30, "calling elevator server"); //method defined below. Args: action client, timeout in sec, description of activity
			}
			//preempt handling or pause if necessary (see basic controller call)





			//Finish -----------------------------------------------

			//set final state using client calls - if you did preempt handling before, put a check here so don't override that


			//log result and set actionlib server state appropriately
			behaviors::ClimbResult result;

			if(preempted_) {
				ROS_WARN("%s: Finished - Preempted", action_name_.c_str());
				result.timed_out_ = false;
				result.success = false;
				as_.setPreempted(result);
			}
			else if(timed_out_) {
				ROS_WARN("%s: Finished - Timed Out", action_name_.c_str());
				result.timed_out_ = true;
				result.success = false;
				as_.setSucceeded(result); //timed out is encoded as succeeded b/c actionlib doesn't have a timed out state
			}
			else { //implies succeeded
				ROS_INFO("%s: Finished - Succeeded", action_name_.c_str());
				result.timed_out_ = false;
				result.success = true;
				as_.setSucceeded(result);
			}

			return;

		}


		void waitForActionlibServer(auto &action_client, double timeout, const std::string &activity)
			//activity is a description of what we're waiting for, e.g. "waiting for mechanism to extend" - helps identify where in the server this was called (for error msgs)
		{
			double request_time = ros::Time::now().toSec();

			//wait for actionlib server to finish
			std::string state;
			while(!preempted_ && !timed_out_ && ros::ok())
			{
				state = action_client.getState().toString();

				if(state == "PREEMPTED") {
					ROS_ERROR_STREAM(action_name_ << ": external actionlib server returned preempted_ during " << activity);
					preempted_ = true;
				}
				//check timeout - note: have to do this before checking if state is SUCCEEDED since timeouts are reported as SUCCEEDED
				else if (ros::Time::now().toSec() - request_time > timeout || //timeout from what this file says
						(state == "SUCCEEDED" && !action_client.getResult()->success)) //server times out by itself
				{
					ROS_ERROR_STREAM(action_name_ << ": external actionlib server timed out during " << activity);
					timed_out_ = true;
					action_client.cancelGoalsAtAndBeforeTime(ros::Time::now());
				}
				else if (state == "SUCCEEDED") { //must have succeeded since we already checked timeout possibility
					break; //stop waiting
				}
				//checks related to this file's actionlib server
				else if (as_.isPreemptRequested() || !ros::ok()) {
					ROS_ERROR_STREAM(action_name_ << ": preempted_ during " << activity);
					preempted_ = true;
				}
				else if (ros::Time::now().toSec() - start_time_ > server_timeout_) {
					ROS_ERROR_STREAM(action_name_ << ": timed out during " << activity);
					timed_out_ = true;
					action_client.cancelGoalsAtAndBeforeTime(ros::Time::now());
				}
				else { //if didn't succeed and nothing went wrong, keep waiting
					ros::spinOnce();
					r.sleep();
				}
			}
		}
};

int main(int argc, char** argv) {
	//create node
	ros::init(argc, argv, "climber_server");

	//get config values
	ros::NodeHandle n;

        double server_timeout = 10;
        double wait_for_server_timeout = 10;

	/* e.g.
	//ros::NodeHandle n_params_intake(n, "actionlib_cargo_intake_params"); //node handle for a lower-down namespace

	if (!n.getParam("/teleop/teleop_params/linebreak_debounce_iterations", linebreak_debounce_iterations))
		ROS_ERROR("Could not read linebreak_debounce_iterations in intake_server");

	if (!n.getParam("/actionlib_params/wait_for_server_timeout", wait_for_server_timeout))
		ROS_ERROR("Could not read wait_for_server_timeout_ in intake_sever");

	if (!n_params_intake.getParam("roller_power", roller_power))
		ROS_ERROR("Could not read roller_power in cargo_intake_server");
	*/

	//create the actionlib server
	ClimberAction climber_action("climber_server", server_timeout, wait_for_server_timeout);

	ros::AsyncSpinner Spinner(2);
	Spinner.start();
	ros::waitForShutdown();
	return 0;
}
