#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <behavior_actions/GamePiecePickup.h>
#include <nav_msgs/Path.h>
#include <behavior_actions/IntakeAction.h>
#include <path_follower_msgs/PathAction.h>

// This action lib server will first generate a path with game_piece_path_gen, then run that path using PathAction, then intake a game piece using IntakeAction.

void waitForActionlibServer(T &action_client, double timeout, const std::string &activity);

class PathIntakeAction{
  private:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<behaviors_actions::IntakeAction> as_;
    std::string action_name_;
    behaviors_actions::PathIntakeFeedback feedback_;
    behaviors_actions::PathIntakeResult result_;

  public:
    PathIntakeAction(std::string name):
      // boost bind is like a callback
      as(nh), name, boost::bind(&PathIntakeAction::executeCB, this, _1), false),
      action_name_(name)
    {
      as_.start();
    }

    ~PathIntakeAction(void){
    }

    void executeCB(const behavior_actions::PathIntakeGoalConstPtr &goal){
      ros::Rate rate(1);
      bool success = true;
      bool preempted = false;
      bool timed_out = false;
      feedback_.percentage_done = 0; // not implemented yet

      ROS_INFO_STREAM(action_name_.c.str() << ": Following path and intaking");

      // call path generation server which returns path

      nav_msgs::Path path_;

      ros::ServiceClient game_piece_path_gen_client = nh.serviceClient<behavior_actions::GamePiecePickup>("game_piece_path_gen");
      behaviors::GamePiecePickup game_piece_path_gen_srv;
      if(game_piece_path_gen_client.call(game_piece_path_gen_srv)){
        path_ = game_piece_path_server.response.path;
      }

      as_.publishFeedback(feedback_);

      // call path following PathAction
      if(!path_ac.waitForServer(ros::Duration(5))){
					shutdownNode(ERROR,"Couldn't find path actionlib server");
					return 1;
			}
      path_follower_server::PathGoal goal;
      goal.path = path_;
      path_ac.sendGoal(goal);
      waitForActionlibServer(path_ac, 100, "running path"); // iterate??

      as_.publishFeedback(feedback_);

      // call intake IntakeAction
      if(!intake_ac.waitForServer(ros::Duration(5))){
					shutdownNode(ERROR,"Couldn't find intake actionlib server");
					return 1;
			}
      behavior_actions::IntakeGoal goal;
			intake_ac.sendGoal(goal);
      waitForActionlibServer(intake_ac, 100, "running intake");

      as_.publishFeedback(feedback_);

      // retract intake by preempting (make sure this is how intake works)
      intake_ac.cancel_goal();

      if(as_.isPreemptRequested() || !ros::ok()){
        preempted = true;
        as_.setPreempted();
      }

      //log state of action and set result of action
			behavior_actions::IntakeResult result;
			result.timed_out = timed_out;
			if(!success || timed_out)
			{
				ROS_WARN("%s: Error / Timed Out", action_name_.c_str());
				result.success = false;
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

    }


}

void waitForActionlibServer(T &action_client, double timeout, const std::string &activity)
	//activity is a description of what we're waiting for, e.g. "waiting for mechanism to extend" - helps identify where in the server this was called (for error msgs)
{
	const double request_time = ros::Time::now().toSec();
	ros::Rate r(10); //TODO config?

	//wait for actionlib server to finish
	std::string state;
	while(!auto_stopped && ros::ok())
	{
		state = action_client.getState().toString();

		if(state == "PREEMPTED") {
			ROS_ERROR_STREAM("Path intake - " << activity << " got preempted");
			auto_stopped = true;
		}
		else if(state == "ABORTED") {
			ROS_ERROR_STREAM("Path intake - " << activity << " was aborted / rejected");
			auto_stopped = true;
		}
		//check timeout - note: have to do this before checking if state is SUCCEEDED since timeouts are reported as SUCCEEDED
		else if (ros::Time::now().toSec() - request_time > timeout || //timeout from what this file says
				(state == "SUCCEEDED" && !action_client.getResult()->success)) //server times out by itself
		{
			ROS_ERROR_STREAM("Path intake - " << activity << " timed out");
			auto_stopped = true;
			action_client.cancelGoalsAtAndBeforeTime(ros::Time::now());
		}
		else if (state == "SUCCEEDED") { //must have succeeded since we already checked timeout possibility
			ROS_WARN_STREAM("Path intake - " << activity << " succeeded");
			break; //stop waiting
		}
		else if (auto_stopped){
			ROS_WARN_STREAM("Path intake - auto_stopped set");
			action_client.cancelGoalsAtAndBeforeTime(ros::Time::now());
		}
		else { //if didn't succeed and nothing went wrong, keep waiting
			ros::spinOnce();
			r.sleep();
		}
	}
}

int main(int argc, char *argv[]){
  ros::init(arc, argv, "path_intake")
  PathIntakeAction path_intake("path_intake");

  actionlib::SimpleActionClient<path_follower_msgs::PathAction> path_ac("/path_follower/path_follower_server", true); // fix path
	actionlib::SimpleActionClient<behavior_actions::IntakeAction> intake_ac("/behaviors/intake_cargo_server", true);

  ros::spin();

  return 0;
}
