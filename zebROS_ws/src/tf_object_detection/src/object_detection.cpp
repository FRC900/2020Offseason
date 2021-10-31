#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "field_obj/Detection.h"
#include "field_obj/TFDetection.h"
#include "field_obj_tracker/convert_coords.h"
#include "field_obj_tracker/objtype.hpp"

#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <std_msgs/String.h>
#include <random>

ros::Publisher pub;

sensor_msgs::CameraInfo caminfo;
bool caminfovalid {false};

// Capture camera info published about the camera - needed for screen to world to work
void camera_info_callback(const sensor_msgs::CameraInfoConstPtr &info)
{
	caminfo = *info;
	caminfovalid = true;
}

// Get the most useful depth value in the cv::Mat depth contained within
// the supplied bounding rectangle
double avgOfDepthMat(const cv::Mat& depth, const cv::Rect& bound_rect, int k = 3, float tolerance = 1e-2)
{
	// TODO figure out why the difference is sometimes nan

	// setup randomizing (to initialize k-means)
	std::random_device seeder;
	std::mt19937 engine(seeder());
	std::uniform_int_distribution<int> distX(bound_rect.tl().x+1, bound_rect.br().x-1);
	std::uniform_int_distribution<int> distY(bound_rect.tl().y+1, bound_rect.br().y-1);

	// initialize arrays (and a vector) for k-means
	double centroids[k];
	double prevCentroids[k];
	std::vector<float> clusters[k];

	// initialize random centroids
	for (int i = 0; i < k; i++) {
		// assign random depth values to centroids (Forgy method of initializing k-means)
		centroids[i] = depth.at<float>(distX(engine), distY(engine));
	}

	while (true) { // once the algorithm converges this returns
		for (int j = bound_rect.tl().y+1; j < bound_rect.br().y; j++) // for each row
		{
			const float *ptr_depth = depth.ptr<float>(j);

			for (int i = bound_rect.tl().x+1; i < bound_rect.br().x; i++) // for each pixel in row
			{
				if (!(isnan(ptr_depth[i]) || isinf(ptr_depth[i]) || (ptr_depth[i] <= 0)))
				{
					// Calculate which centroid/mean the current pixel is closest to
					float diffs[k];
					for (int c = 0; c < k; c++) {
						diffs[c] = abs(centroids[c] - ptr_depth[i]);
					}
					clusters[std::distance(diffs, std::min_element(diffs, diffs+k))].push_back(ptr_depth[i]);
					// NOTE can probably implement this faster with modifying min_element to subtract ptr_depth[i] for us
				}
			}
		}

		// Recalculate centroids using the average of the cluster closest to a centroid
		for (int i = 0; i < k; i++) {
			double sum = 0;
			for (float f : clusters[i]) {
				sum += f;
			}
			centroids[i] = sum / (double)clusters[i].size();
			clusters[i].clear(); // Clear clusters
		}

		// Calculate and print the difference between the current and previous centroids
		// this lets us see when the difference is very low (in which case the algorithm will be done)
		float diff = 0;
		for (int i = 0; i < k; i++) {
			diff += abs(centroids[i] - prevCentroids[i]);
		}
		ROS_INFO_STREAM("diff: " << diff);

		// If the difference is less than the tolerance, return the closest centroid
		if (diff <= tolerance) {
			return *std::min_element(centroids, centroids+k);
		}

		// If the above statement didn't return, copy centroids to prevCentroids and
		// re-run the algorithm
		memcpy(prevCentroids, centroids, sizeof(prevCentroids));
	}
}


// For each object in objDetectionMsg, look up the depth reported in depthMsg at the center of the
// object's bounding rectangle. Use that to convert from 2D screen coordinates to 3D world coordinates
void callback(const field_obj::TFDetectionConstPtr &objDetectionMsg, const sensor_msgs::ImageConstPtr &depthMsg)
{
	if (!caminfovalid)
		return;

	cv_bridge::CvImageConstPtr cvDepth = cv_bridge::toCvShare(depthMsg, sensor_msgs::image_encodings::TYPE_32FC1);

	// Initialize published message with header info from objDetectionMsg
	field_obj::Detection out_msg;
	out_msg.header = objDetectionMsg->header;

	// Create objects needed to convert from 2d screen to 3d world coords
	image_geometry::PinholeCameraModel model;
	model.fromCameraInfo(caminfo);
	const ConvertCoords cc(model);
	// Iterate over each object. Convert from camera to world coords.
	for (const auto &camObject : objDetectionMsg->objects)
	{
		// Create an output object, copy over info from the object detection info
		field_obj::Object worldObject;
		worldObject.id = camObject.label;
		worldObject.confidence = camObject.confidence;
		// Generate a bounding rect (in camera coords) from the camera object
		const cv::Point rectTL(camObject.tl.x, camObject.tl.y);
		const cv::Point rectBR(camObject.br.x, camObject.br.y);
		const cv::Rect  rect(rectTL, rectBR);

		// Create a small bounding rectangle to sample depths from the center
		// of the detected object.  Note, this will only work with power cells
		// and other objects which don't have holes in the middle (mmmm ... donuts!)
		const cv::Point2f objRectCenter = 0.5 * (rectTL + rectBR);
		const cv::Point2f objCenterBounds(3, 3);
		const cv::Rect  objCenterRect(objRectCenter - objCenterBounds, objRectCenter + objCenterBounds);

		// Get the distance to the object by sampling the depth image at the center of the
		// object's bounding rectangle.
		const double objDistance = avgOfDepthMat(cvDepth->image, objCenterRect);
		if (objDistance < 0)
		{
			ROS_ERROR_STREAM("Depth of object at " << objRectCenter << " with bounding rect " << objCenterRect << " objDistance < 0 : " << objDistance);
			continue;
		}
		const cv::Point3f world_coord_scaled = cc.screen_to_world(rect, worldObject.id, objDistance);

		// Convert from camera_optical_frame to camera_frame - could do this
		// using transforms in the future?
		worldObject.location.x = world_coord_scaled.z;
		worldObject.location.y = -world_coord_scaled.x;
		worldObject.location.z = world_coord_scaled.y;
		worldObject.angle = atan2(worldObject.location.y, worldObject.location.x) * 180. / M_PI;

		// Add the 3d object info to the list of objects in the output message
		out_msg.objects.push_back(worldObject);
	}

	pub.publish(out_msg);

	if (out_msg.objects.size() > 0)
	{
		//Transform between goal frame and odometry/map.
		static tf2_ros::TransformBroadcaster br;
		for(size_t i = 0; i < out_msg.objects.size(); i++)
		{
			geometry_msgs::TransformStamped transformStamped;

			transformStamped.header.stamp = out_msg.header.stamp;
			transformStamped.header.frame_id = out_msg.header.frame_id;
			std::stringstream child_frame;
			child_frame << out_msg.objects[i].id;
			child_frame << "_";
			child_frame << i;
			transformStamped.child_frame_id = child_frame.str();

			transformStamped.transform.translation.x = out_msg.objects[i].location.x;
			transformStamped.transform.translation.y = out_msg.objects[i].location.y;
			transformStamped.transform.translation.z = out_msg.objects[i].location.z;

			// Can't detect rotation yet, so publish 0 instead
			tf2::Quaternion q;
			q.setRPY(0, 0, out_msg.objects[i].angle);

			transformStamped.transform.rotation.x = q.x();
			transformStamped.transform.rotation.y = q.y();
			transformStamped.transform.rotation.z = q.z();
			transformStamped.transform.rotation.w = q.w();

			br.sendTransform(transformStamped);
		}
	}
}


void testAvgOfDepthMatCallback(const std_msgs::String::ConstPtr& msg) {
	// the message will be a filepath to an image file for testing
	ROS_INFO_STREAM("Received " << msg->data);
	cv::Mat depth = cv::imread(msg->data, cv::IMREAD_GRAYSCALE); // read image as grayscale
	depth.convertTo(depth, 5); // type 32FC1, the constant wasn't coming up
	depth /= 128.0; // 0-255 -> ~0-2 (meters)
	depth += 0.5; // ~0-2 -> ~0-2.5

	// Add random noise
	cv::Mat noise(depth.size(), depth.type());
	cv::randn(noise, 0, 0.1);
	depth += noise;

 	// Calculate the most useful depth and print it
	cv::Rect depth_rect = cv::Rect(0, 0, depth.size().width, depth.size().height);
	ROS_INFO_STREAM("Calculated depth is " << avgOfDepthMat(depth, depth_rect));
}


int main (int argc, char **argv)
{
	ros::init(argc, argv, "tf_object_screen_to_world");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);

	// Create a filter subscriber to camera depth info
	std::unique_ptr<image_transport::SubscriberFilter> depth_sub;
	depth_sub = std::make_unique<image_transport::SubscriberFilter>(it, "/zed_objdetect/depth/depth_registered", 1);

	// And another filer subscriber to the TF object detection
	std::unique_ptr<message_filters::Subscriber<field_obj::TFDetection>> obsub;
	obsub = std::make_unique<message_filters::Subscriber<field_obj::TFDetection>>(nh, "obj_detection_msg", 1);

	// Create a synchronizer which combines the two - it calls the callback function when it matches
	// up a message from each subscriber which have the (approximately) same timestamps
	// TODO - try this with an exact synchronizer?
	typedef message_filters::sync_policies::ApproximateTime<field_obj::TFDetection, sensor_msgs::Image> ObjDepthSyncPolicy;
	std::unique_ptr<message_filters::Synchronizer<ObjDepthSyncPolicy>> obj_depth_sync;
	obj_depth_sync = std::make_unique<message_filters::Synchronizer<ObjDepthSyncPolicy>>(ObjDepthSyncPolicy(10), *obsub, *depth_sub);

	obj_depth_sync->registerCallback(boost::bind(callback, _1, _2));

	// Set up a simple subscriber to capture camera info
	ros::Subscriber camera_info_sub_ = nh.subscribe("/zed_objdetect/left/camera_info", 2, camera_info_callback);

	// And a publisher to published converted 3d coords
	pub = nh.advertise<field_obj::Detection>("object_detection_world", 2);

	// Add a subscriber to subscribe to testing messages
	ros::Subscriber sub = nh.subscribe("test_depth", 10, testAvgOfDepthMatCallback);

	ros::spin();
}
