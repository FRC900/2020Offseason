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

// Find the median of a continuous cv::Mat
float findMedianOfMat(cv::Mat mat) {
	float median = 0;
	if (mat.isContinuous()) {
		// copy matrix data to a vector
		std::vector<float> vec;
		mat = mat.reshape(0, 1);
		mat.copyTo(vec);
		// remove 255 (when this is called, values that are masked are 255)
		vec.erase(std::remove(vec.begin(), vec.end(), 255), vec.end());
		// sort vector
		std::sort(vec.begin(), vec.end()); // forgot this earlier
		if ((vec.size() % 2) != 0) { // if odd
			auto it = vec.begin() + vec.size()/2;
			std::nth_element(vec.begin(), it, vec.end());
			median = vec[vec.size()/2];
		} else { // if even
			auto it = vec.begin() + vec.size()/2;
			std::nth_element(vec.begin(), it, vec.end());
			median += vec[vec.size()/2];
			it = vec.begin() + vec.size()/2 - 1;
			std::nth_element(vec.begin(), it, vec.end());
			median += vec[vec.size()/2];
			median = ((float)median/2.0d);
		}
	}
	return median;
}


// Get the most useful depth value in the cv::Mat depth contained within
// the supplied bounding rectangle
double usefulDepthMat(const cv::Mat& depth_, const cv::Rect& bound_rect, bool debug = false)
{
	// Crop depth to region of interest
	cv::Mat depth = depth_(bound_rect);

	// convert depth to a 0-255 grayscale image (for contour finding)
	cv::Mat depthDifferentFormat;
	cv::Mat zeros = depth==0; // no depth is 0 with the ZED
	depth.copyTo(depthDifferentFormat);
	cv::normalize(depthDifferentFormat, depthDifferentFormat, 0, 128, cv::NORM_MINMAX); // 0-128 because outliers will be 255 (and we want a clear background)
	depthDifferentFormat.setTo(255, zeros); // set zeros (no depth) to 255
	depthDifferentFormat.convertTo(depthDifferentFormat, CV_8UC1, 1);

	if (debug) {
		cv::imshow("Depth", depthDifferentFormat);
		cv::waitKey(0);
	}

	// Find the median of the image
	float median = findMedianOfMat(depthDifferentFormat);

	// use adaptive thresholding to convert image to black and white for finding
	// contours
	cv::Mat threshOutput;
	int blockSize = depthDifferentFormat.size().area() / 25;
	blockSize = blockSize % 2 == 0 ? blockSize + 1 : blockSize;
	cv::adaptiveThreshold(depthDifferentFormat, threshOutput, 1, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY_INV, blockSize, 0);

	// uncomment for thresholding using the median
	// cv::Mat threshOutput;
	// cv::threshold(depthDifferentFormat, threshOutput, median, 1, cv::THRESH_BINARY_INV);

	// find contours
	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours(threshOutput, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

	// create a mask
	cv::Mat mask = cv::Mat::zeros(threshOutput.size(), CV_8UC1);
	int largestContourIndex;
	float largestArea = 0;
	for (size_t i = 0; i < contours.size(); i++) {
		std::vector<cv::Point> contour = contours[(int)i];
		cv::Rect rect = cv::boundingRect(contour);
		if (rect.area() > largestArea) {
			largestArea = rect.area();
			largestContourIndex = (int)i;
		}
	}
	// draw the largest contour onto the mask
	cv::Scalar color = cv::Scalar(255);
	cv::drawContours(mask, contours, largestContourIndex, color, -1, cv::LINE_8, hierarchy, 0);

	// make a new image for the original depth cut out by the mask, and fill it with 999
	// 999 = ignore value, see line 139
	cv::Mat masked = cv::Mat::ones(depth.size(), depth.type()) * 999;
	depth.copyTo(masked, mask);

	// if debug is enabled,
	if (debug) {
		// show the masked image
		cv::Mat destination = cv::Mat::zeros(depth.size(), depth.type());
		depth.copyTo(destination, mask);

		cv::normalize(destination, destination, 0, 255, cv::NORM_MINMAX);
		destination.convertTo(destination, CV_8UC1);
		cv::imshow("Masked", destination);
		cv::waitKey(0);
	}

	// copy matrix data to a vector
	std::vector<float> vec;
	masked = masked.reshape(0, 1);
	masked.copyTo(vec);
	// remove 999 values from the mask
	vec.erase(std::remove(vec.begin(), vec.end(), 999), vec.end());
	// and 0 values (no depth)
	vec.erase(std::remove(vec.begin(), vec.end(), 0), vec.end());
	// sort vector
	std::sort(vec.begin(), vec.end());

	return vec[0]; // return lowest value. Ideally, we'd want to throw out the bottom
	// <some number>%, but that messes up depth calculation for things like spheres.

	/* Test results:
	[ INFO] [1635948213.029497061]: Received /home/ubuntu/2020Offseason/zebROS_ws/src/tf_object_detection/test_bitmaps/cropped_goal_8.jpg
	[ INFO] [1635948213.689598629]: Calculated depth is 113.001
	[ INFO] [1635948229.253951900]: Received /home/ubuntu/2020Offseason/zebROS_ws/src/tf_object_detection/test_bitmaps/cropped_goal_behind_power_cell.png
	[ INFO] [1635948229.870174338]: Calculated depth is 138.001
	[ INFO] [1635948236.848476178]: Received /home/ubuntu/2020Offseason/zebROS_ws/src/tf_object_detection/test_bitmaps/cropped_goal.png
	[ INFO] [1635948237.445102845]: Calculated depth is 113.001
	[ INFO] [1635948248.428153064]: Received /home/ubuntu/2020Offseason/zebROS_ws/src/tf_object_detection/test_bitmaps/cropped_loading_bay.png
	[ INFO] [1635948249.227361594]: Calculated depth is 141.001
	[ INFO] [1635948255.684437760]: Received /home/ubuntu/2020Offseason/zebROS_ws/src/tf_object_detection/test_bitmaps/cropped_low_goal.png
	[ INFO] [1635948256.286312490]: Calculated depth is 92.001
	[ INFO] [1635948266.234706198]: Received /home/ubuntu/2020Offseason/zebROS_ws/src/tf_object_detection/test_bitmaps/cropped_power_cell.png
	[ INFO] [1635948266.316279430]: Calculated depth is 0.001
	[ INFO] [1635948273.702984310]: Received /home/ubuntu/2020Offseason/zebROS_ws/src/tf_object_detection/test_bitmaps/goal_behind_power_cell.png
	[ INFO] [1635948275.086796499]: Calculated depth is 0.001
	[ INFO] [1635948302.568242461]: Received /home/ubuntu/2020Offseason/zebROS_ws/src/tf_object_detection/test_bitmaps/goal_with_power_cell_in_front.jpg
	[ INFO] [1635948302.598558216]: Calculated depth is 0.001
	[ INFO] [1635948312.950812216]: Received /home/ubuntu/2020Offseason/zebROS_ws/src/tf_object_detection/test_bitmaps/test_weirdness.jpg
	[ INFO] [1635948312.980837687]: Calculated depth is 73.001
	*/
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
		const double objDistance = usefulDepthMat(cvDepth->image, objCenterRect);
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


void testUsefulDepthMatCallback(const std_msgs::String::ConstPtr& msg) {
	// the message will be a filepath to an image file for testing
	ROS_INFO_STREAM("Received " << msg->data);
	cv::Mat depth;
	if (msg->data.back() == 'r') { // likely a .ex`r` file, treating it as one
		// make sure to set outliers to zero in the file
		depth = cv::imread(msg->data, cv::IMREAD_ANYCOLOR | cv::IMREAD_ANYDEPTH); // read image
		depth.convertTo(depth, CV_32FC1);
		std::vector<cv::Mat> channels(3);
		cv::split(depth, channels);
		depth = channels[0];
	} else {
		depth = cv::imread(msg->data, cv::IMREAD_GRAYSCALE); // read image as grayscale
		depth.convertTo(depth, CV_32FC1);
		// Add random noise
		cv::Mat noise(depth.size(), depth.type());
		cv::randn(noise, 0, 5);
		cv::Mat double_noise(depth.size(), depth.type());
		cv::randn(double_noise, 1, 0.1);
		depth += 1; // with the ZED, a zero means no depth
	}

 	// Calculate the most useful depth and print it
	cv::Rect depth_rect = cv::Rect(0, 0, depth.size().width, depth.size().height);
	ROS_INFO_STREAM("Calculated depth is " << usefulDepthMat(depth, depth_rect, true));
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
	ros::Subscriber sub = nh.subscribe("test_depth", 10, testUsefulDepthMatCallback);

	ros::spin();
}
