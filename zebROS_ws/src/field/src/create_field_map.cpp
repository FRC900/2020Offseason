#include <iostream>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <XmlRpcValue.h>

double field_width, field_height, border, inches_per_pixel; //field dimensions in inches
int width, height;

cv::Point map_to_image(cv::Point map_coord)
{
	double image_x = map_coord.x / inches_per_pixel + border;
	double image_y = (height - map_coord.y / inches_per_pixel) + border;
	return cv::Point(image_x, image_y);
}

void drawRotatedRectangle(cv::Mat& image, const cv::Point &centerPoint, const cv::Size &rectangleSize, const double rotationDegrees, const cv::Scalar &color)
{
	// Create the rotated rectangle
	cv::RotatedRect rotatedRectangle(centerPoint, rectangleSize, rotationDegrees);

	// We take the edges that OpenCV calculated for us
	cv::Point2f vertices2f[4];
	rotatedRectangle.points(vertices2f);

	// Convert them so we can use them in a fillConvexPoly
	cv::Point vertices[4];
	for(int i = 0; i < 4; ++i){
		vertices[i] = vertices2f[i];
	}

	// Now we can fill the rotated rectangle with our specified color
	cv::fillConvexPoly(image, vertices, 4, color);
}

void drawTriangle(cv::Mat& image, const cv::Point vertices[3], const cv::Scalar &color)
{
	// Convert map coordinates to image coordinates
	cv::Point image_vertices[3];
	for(int i = 0; i < 3; ++i){
		image_vertices[i] = map_to_image(vertices[i]);
	}

	// Now we can fill the triangle with our specific color
	cv::fillConvexPoly(image, image_vertices, 3, color);
}

void drawCircle(cv::Mat& image, const cv::Point &center, const double radius, const cv::Scalar &color)
{
	// Convert map coordinates to image coordinates
	cv::Point image_center = map_to_image(center);

	// Now we can fill the triangle with our specific color
	cv::circle(image, image_center, radius, color, CV_FILLED, 8);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "create_field_map");
  ros::NodeHandle nh_;

	XmlRpc::XmlRpcValue xml_obstacles_list, xml_obstacle;

  if (!nh_.getParam("field_dims/width", field_width)) {
    ROS_ERROR("field dimension not specified");
    return -1;
  }

  if (!nh_.getParam("field_dims/height", field_height)) {
    ROS_ERROR("field dimension not specified");
    return -1;
  }

	if (!nh_.getParam("obstacles", xml_obstacles_list)) {
    ROS_ERROR("failed to load obstacles");
    return -1;
  }

	width = 1200;
	height = width / (field_width / field_height);
	border = width * .05;
	inches_per_pixel = field_width / width;

	// Create mat large enough to hold field plus a border
	cv::Mat image(cv::Size(width + 2 * border, height + 2 * border), CV_8UC1, cv::Scalar(255));
	ROS_INFO_STREAM(image.size() << " " << image.depth() << " " << image.channels());

	// Draw field border
	for (int row = 0; row < image.rows; row++)
	{
		for (int col = 0; col < image.cols; col++)
		{
			const double xpos_in_inches = static_cast<double>(col - border) * inches_per_pixel;
			const double ypos_in_inches = static_cast<double>(row - border) * inches_per_pixel;
			if ((xpos_in_inches < 0) || (ypos_in_inches < 0))
			{
				image.at<uchar>(row, col) = 0;
			}
			if (xpos_in_inches > field_width)
			{
				image.at<uchar>(row, col) = 0;
			}
			if (ypos_in_inches > field_height)
			{
				image.at<uchar>(row, col) = 0;
			}
		}
	}

	for (size_t i = 0; i < (unsigned) xml_obstacles_list.size(); i++) {
		xml_obstacle = xml_obstacles_list[i];
		std::string type = xml_obstacle["type"];

		if(type == "triangle")
		{
			cv::Point vertices[3];
			vertices[0] = cv::Point(xml_obstacle["p1"][0], xml_obstacle["p1"][1]);
			vertices[1] = cv::Point(xml_obstacle["p2"][0], xml_obstacle["p2"][1]);
			vertices[2] = cv::Point(xml_obstacle["p3"][0], xml_obstacle["p3"][1]);

			drawTriangle(image, vertices, cv::Scalar(0,0,0));
		}
		else if(type == "circle")
		{
			cv::Point center = cv::Point(xml_obstacle["center"][0], xml_obstacle["center"][1]);
			
			drawCircle(image, center, xml_obstacle["radius"], cv::Scalar(0,0,0));
		}

  }

	// Calculations for various inputs to stage and map_server
	double meter_per_pixel = (field_width * .0254) / width;
	ROS_INFO_STREAM("meters per pixel: " << meter_per_pixel);
	ROS_INFO_STREAM("width x height: " << meter_per_pixel * image.cols << " " << meter_per_pixel * image.rows);
	std::cout << "pose " << (meter_per_pixel * image.cols) / 2. - border * meter_per_pixel << " " << (meter_per_pixel * image.rows) / 2. - border * meter_per_pixel << std::endl;
	cv::imwrite("2020Field.png", image);
	cv::imshow("image", image);
	cv::waitKey(0);

	return 0;
}
