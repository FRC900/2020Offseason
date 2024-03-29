// Simple node to generate a fake goal detection message in simulation
// Reads base_marker_detection message from stage ros, converts to our custom
// goal detect message. Adds noise to x and y positions of the converted detections
#include <random>
#include <ros/ros.h>
#include <marker_msgs/MarkerDetection.h>
#include "field_obj/Detection.h"

class FakeGoalDetection
{
	public:
		FakeGoalDetection(ros::NodeHandle &n)
			: rd_{}
			, gen_{rd_()}
			, covariance_(0.0004)
			, sub_(n.subscribe("base_marker_detection", 2, &FakeGoalDetection::cmdVelCallback, this))
			, pub_(n.advertise<field_obj::Detection>("goal_detect_msg", 2))

		{
			n.param("covariance", covariance_, covariance_);
			normalDistribution_ = std::normal_distribution<double>{0, sqrt(covariance_)};
		}

		// Translate stage base_marker_detection into our custom goal detection message
		void cmdVelCallback(const marker_msgs::MarkerDetectionConstPtr &msgIn)
		{
			field_obj::Detection msgOut;
			msgOut.header = msgIn->header;
			for(size_t i = 0; i < msgIn->markers.size(); i++)
			{
				if (msgIn->markers[i].ids[0] == -1) // stage publishes odom as marker -1
					continue;                       // ignore it here
				field_obj::Object dummy;

				const auto &p = msgIn->markers[i].pose.position;
				dummy.location.x = p.x + normalDistribution_(gen_);
				dummy.location.y = p.y + normalDistribution_(gen_);
				dummy.location.z = p.z + normalDistribution_(gen_);
				dummy.angle = atan2(dummy.location.y, dummy.location.x) * 180. / M_PI;
				dummy.confidence = msgIn->markers[i].ids_confidence[0];
				dummy.id = std::to_string(msgIn->markers[i].ids[0]);
				msgOut.objects.push_back(dummy);
			}
			pub_.publish(msgOut);
		}

	private:
		std::random_device rd_;
		std::mt19937 gen_;
		std::normal_distribution<double> normalDistribution_;
		double covariance_;
		ros::Subscriber sub_;
		ros::Publisher  pub_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fake_goal_detect");

	ros::NodeHandle n;
	FakeGoalDetection fakeGoalDetection(n);

	ros::spin();
	return 0;
}
