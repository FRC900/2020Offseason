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

#include <std_msgs/String.h> // for testing depth detection on the `test_depth` topic
#include <random> // for Forgy method of initializing k-means for depth detection

ros::Publisher pub;

sensor_msgs::CameraInfo caminfo;
bool caminfovalid {false};

// K_MEANS = k-means, Forgy method of initializing (k-means++ would likely be better, but Forgy is working ok for now)
// CONTOURS = find depth by finding contours and taking the lowest value of the biggest contour
// CONTOURS_NON_ADAPTIVE = contours, but using thresholding based on the median instead of adaptive thresholding
enum DepthCalculationAlgorithm { K_MEANS, CONTOURS, CONTOURS_NON_ADAPTIVE };

constexpr float depth_epsilon = 0.01; // If depth values are greater than this value (in meters) away from each other, they are treated as unique. See `kMeansDepthMat`.
constexpr float percent_from_bottom_contours = 0; // The contours algorithm finds the biggest contour in the image and returns the lowest value, excluding this percent of low values (outliers).
																									// For example, if this is 0, the contours algorithm will return the lowest value in the largest contour.
																									// If this is 0.01, the algorithm will throw out the lowest 1% of data and then return the lowest value.
constexpr DepthCalculationAlgorithm algorithm = CONTOURS; // The algorithm used to find depth data

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
		if ((vec.size() % 2) != 0) { // if odd
			auto it = vec.begin() + vec.size()/2;
			std::nth_element(vec.begin(), it, vec.end()); // sort vector so that the middle value is sorted, but not any of the other values
			median = vec[vec.size()/2];
		} else { // if even
			auto it = vec.begin() + vec.size()/2;
			std::nth_element(vec.begin(), it, vec.end()); // sort vector so that the middle value is sorted, but not any of the other values
			median += vec[vec.size()/2];
			it = vec.begin() + vec.size()/2 - 1;
			std::nth_element(vec.begin(), it, vec.end()); // sort vector so that the value before the middle value is sorted, but not any of the other values
			median += vec[vec.size()/2 - 1];
			median = median/(float)2.0;
		}
	}
	return median;
}

// Get the most useful depth value in the cv::Mat depth contained within
// the supplied bounding rectangle, using contour finding
float contoursDepthMat(const cv::Mat& depth_, const cv::Rect& bound_rect, bool debug = false, bool adaptive = true) {
	if (bound_rect.size().area() == 0) { // if the ROI is zero, return -1 (no depth)
		return -1;
	}

	// Crop depth to region of interest
	cv::Mat depth = depth_(bound_rect);

	// set very large outliers and nan to 0 so they can be removed later. TODO see if the ZED actually reports negative depth
	float nan_ = std::numeric_limits<float>::quiet_NaN();
	cv::Mat inf = depth>=900;
	cv::Mat neg_inf = depth<=-900;
	cv::Mat nan = depth==nan_;
	cv::Mat neg_nan = depth==-nan_;
	depth.setTo(0, inf);
	depth.setTo(0, neg_inf);
	depth.setTo(0, nan);
	depth.setTo(0, neg_nan);

	if (debug) {
		double min, max; // cv::minMaxLoc requires doubles which is why a double is used here. Also, this is only enabled when debug==true.
		cv::minMaxLoc(depth, &min, &max);
		ROS_INFO_STREAM("min: " << min << ", max: " << max);
		cv::Mat dest;
		depth.copyTo(dest);
		cv::normalize(dest, dest, 0, 255, cv::NORM_MINMAX);
		dest.convertTo(dest, CV_8UC1);
		cv::imshow("Depth", dest);
		cv::waitKey(1);
	}

	// convert depth to a 0-255 grayscale image (for contour finding)
	cv::Mat depthDifferentFormat;
	cv::Mat zeros = depth==0; // no depth is 0 with the ZED. May also need to check for inf and nan.
	depth.copyTo(depthDifferentFormat);
	cv::normalize(depthDifferentFormat, depthDifferentFormat, 0, 128, cv::NORM_MINMAX); // 0-128 because outliers will be 255 (and we want a clear background)
	depthDifferentFormat.setTo(255, zeros); // set zeros (no depth) to 255
	depthDifferentFormat.convertTo(depthDifferentFormat, CV_8UC1, 1);

	// Find the median of the image
	float median = findMedianOfMat(depthDifferentFormat);

	// Create a cv::Mat for thresholding output
	cv::Mat threshOutput;

	if (adaptive) {
		// use adaptive thresholding to convert image to black and white for finding
		// contours
		int blockSize = depthDifferentFormat.size().area() / 25;
		blockSize = blockSize > 1 ? (blockSize % 2 == 0 ? blockSize + 1 : blockSize) : 5; // block size must be at least 3 and odd
		cv::adaptiveThreshold(depthDifferentFormat, threshOutput, 1, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY_INV, blockSize, 0);
	} else {
		// thresholding using median
		cv::Mat threshOutput;
		cv::threshold(depthDifferentFormat, threshOutput, median, 1, cv::THRESH_BINARY_INV);
	}

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

	// make a new image for the original depth cut out by the mask, and fill it with std::numeric_limits<float>::max()
	// std::numeric_limits<float>::max() = ignore value, see line 168
	cv::Mat masked = cv::Mat::ones(depth.size(), depth.type()) * std::numeric_limits<float>::max();
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
	// remove std::numeric_limits<float>::max() values from the mask
	vec.erase(std::remove(vec.begin(), vec.end(), std::numeric_limits<float>::max()), vec.end());
	// and 0 values (no depth)
	vec.erase(std::remove(vec.begin(), vec.end(), 0), vec.end());
	// sort vector
	std::sort(vec.begin(), vec.end());

	return (vec.size() != 0 ? vec[(size_t)(percent_from_bottom_contours * vec.size())] : -1); // return lowest value after throwing out the bottom <percent_from_bottom_contours>%.
	// If there are no values, return -1.

	/* Test results (adaptive thresholding):
	[ INFO] [1635948213.029497061]: Received /home/ubuntu/2020Offseason/zebROS_ws/src/tf_object_detection/test_bitmaps/cropped_goal_8.jpg
	[ INFO] [1635948213.689598629]: Calculated depth is 113.001 ✅
	[ INFO] [1635948229.253951900]: Received /home/ubuntu/2020Offseason/zebROS_ws/src/tf_object_detection/test_bitmaps/cropped_goal_behind_power_cell.png
	[ INFO] [1635948229.870174338]: Calculated depth is 138.001 ✅
	[ INFO] [1635948236.848476178]: Received /home/ubuntu/2020Offseason/zebROS_ws/src/tf_object_detection/test_bitmaps/cropped_goal.png
	[ INFO] [1635948237.445102845]: Calculated depth is 113.001 ✅
	[ INFO] [1635948248.428153064]: Received /home/ubuntu/2020Offseason/zebROS_ws/src/tf_object_detection/test_bitmaps/cropped_loading_bay.png
	[ INFO] [1635948249.227361594]: Calculated depth is 141.001 ✅
	[ INFO] [1635948255.684437760]: Received /home/ubuntu/2020Offseason/zebROS_ws/src/tf_object_detection/test_bitmaps/cropped_low_goal.png
	[ INFO] [1635948256.286312490]: Calculated depth is 92.001 ✅
	[ INFO] [1635948266.234706198]: Received /home/ubuntu/2020Offseason/zebROS_ws/src/tf_object_detection/test_bitmaps/cropped_power_cell.png
	[ INFO] [1635948266.316279430]: Calculated depth is 0.001 ✅
	[ INFO] [1635948273.702984310]: Received /home/ubuntu/2020Offseason/zebROS_ws/src/tf_object_detection/test_bitmaps/goal_behind_power_cell.png
	[ INFO] [1635948275.086796499]: Calculated depth is 0.001 **non-cropped version** ❌
	[ INFO] [1635948302.568242461]: Received /home/ubuntu/2020Offseason/zebROS_ws/src/tf_object_detection/test_bitmaps/goal_with_power_cell_in_front.jpg
	[ INFO] [1635948302.598558216]: Calculated depth is 0.001 ❌
	[ INFO] [1635948312.950812216]: Received /home/ubuntu/2020Offseason/zebROS_ws/src/tf_object_detection/test_bitmaps/test_weirdness.jpg
	[ INFO] [1635948312.980837687]: Calculated depth is 73.001 ✅
	*/
}

// Get the most useful depth value in the cv::Mat depth contained within
// the supplied bounding rectangle, using k-means
float kMeansDepthMat(const cv::Mat& depth, const cv::Rect& bound_rect, bool debug = false, size_t maximumK = 3, float tolerance = 1e-3)
{
	// setup randomizing (for initialization of k-means)
	std::random_device seeder;
	std::mt19937 engine(seeder());

	// Start of checking if k is too high

	// Convert cropped depth matrix to vector
	std::vector<float> vec;
	cv::Mat mat = depth(bound_rect);
	for (int i = 0; i < mat.rows; i++) {
    vec.insert(vec.end(), mat.ptr<float>(i), mat.ptr<float>(i)+mat.cols*mat.channels());
  }

	// Find unique (defined as >`depth_epsilon` meters away) values in the vector
	std::sort(vec.begin(), vec.end());
	auto last = std::unique(vec.begin(), vec.end(), [](float first, float second){ return fabs(second - first)<depth_epsilon; });
	vec.erase(last, vec.end());

	// Set k to maximumK if there are more than maximumK unique values, otherwise set it to the number of unique values
	size_t k = std::min(vec.size(), maximumK);

	// End of checking if k is too high

	// initialize arrays (and a vector) for k-means
	float centroids[k];
	float prevCentroids[k];
	std::vector<float> clusters[k];

	// Pick centroids
	std::sample(vec.begin(), vec.end(), centroids, k, engine);

	// Print centroids
	if (debug) {
		ROS_INFO_STREAM("k = " << k);
		ROS_INFO_STREAM("Centroids: ");
		for (size_t i = 0; i < k; i++) {
			ROS_INFO_STREAM(centroids[i]);
		}
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
					size_t closestCentroidIndex = -1;
					float smallestDiff = std::numeric_limits<float>::max();
					for (size_t c = 0; c < k; c++) {
						float diff = fabs(centroids[c] - ptr_depth[i]);
						if (diff < smallestDiff) {
							closestCentroidIndex = c;
							smallestDiff = diff;
						}
					}
					// Append the pixel's value to the cluster corresponding to that centroid
					clusters[closestCentroidIndex].push_back(ptr_depth[i]);
				}
			}
		}

		// Recalculate centroids using the average of the cluster closest to each centroid
		// (or set the centroid to std::numeric_limits<float>::max() if there are no values in the cluster)
		for (size_t i = 0; i < k; i++) {
			float sum = 0;
			for (float f : clusters[i]) {
				sum += f;
			}
			if (clusters[i].size() != 0) {
				centroids[i] = sum / (float)clusters[i].size();
			} else { // If the centroid's cluster has no values, set it to std::numeric_limits<float>::max() (basically remove it)
				centroids[i] = std::numeric_limits<float>::max();
			}
			clusters[i].clear(); // Clear clusters
		}

		// Calculate and print the difference between the current and previous centroids
		// this lets us see when the difference is very low (in which case the algorithm will be done)
		float diff = 0;
		for (size_t i = 0; i < k; i++) {
			diff += fabs(centroids[i] - prevCentroids[i]);
		}
		if (debug) {
			ROS_INFO_STREAM("diff: " << diff);
		}

		// If the difference is less than the tolerance, return the closest centroid
		if (diff <= tolerance) {
			return *std::min_element(centroids, centroids+k);
		}

		// If the above statement didn't return, copy centroids to prevCentroids and
		// re-run the algorithm
		memcpy(prevCentroids, centroids, sizeof(prevCentroids));
	}
}

// Get the most useful depth value in the cv::Mat depth contained within
// the supplied bounding rectangle
float usefulDepthMat(const cv::Mat& depth, const cv::Rect& bound_rect, bool debug = false, DepthCalculationAlgorithm algorithm = CONTOURS, int k = 3, float tolerance = 1e-3)
{
	switch (algorithm) {
		case CONTOURS:
			return contoursDepthMat(depth, bound_rect, debug);
		case CONTOURS_NON_ADAPTIVE:
			return contoursDepthMat(depth, bound_rect, debug, false);
		case K_MEANS:
			return kMeansDepthMat(depth, bound_rect, debug, k, tolerance);
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

		// Find the center of the detected object
		const cv::Point2f objRectCenter = 0.5 * (rectTL + rectBR);

		// Get the distance to the object by finding contours within the depth data inside the
		// object's bounding rectangle.
		const float objDistance = usefulDepthMat(cvDepth->image, rect, false, algorithm);
		if (objDistance < 0)
		{
			ROS_ERROR_STREAM("Depth of object at " << objRectCenter << " with bounding rect " << rect << " objDistance < 0 : " << objDistance);
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


// A callback function for testing usefulDepthMat. This is for a subscriber that
// subscribes to the `test_depth` topic.
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

 	// Calculate the most useful depth using contour finding (default algorithm) and print it
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

	// obj_depth_sync->setMaxIntervalDuration(ros::Duration(1, 0)); // for testing rosbags
	obj_depth_sync->registerCallback(boost::bind(callback, _1, _2));

	// Set up a simple subscriber to capture camera info
	ros::Subscriber camera_info_sub_ = nh.subscribe("/zed_objdetect/left/camera_info", 2, camera_info_callback);

	// And a publisher to published converted 3d coords
	pub = nh.advertise<field_obj::Detection>("object_detection_world", 2);

	// Add a subscriber to subscribe to testing messages
	// Example of message to send: rostopic pub /test_depth std_msgs/String /home/ubuntu/2020Offseason/zebROS_ws/src/tf_object_detection/test_bitmaps/cropped_goal_behind_power_cell.png
	ros::Subscriber sub = nh.subscribe("test_depth", 10, testUsefulDepthMatCallback);

	ros::spin();
}
