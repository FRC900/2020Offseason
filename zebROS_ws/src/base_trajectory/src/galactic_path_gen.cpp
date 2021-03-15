#include <ros/ros.h>
#include <math.h>
#include "base_trajectory_msgs/GenerateSpline.h"
#include "base_trajectory_msgs/GalacticPathGen.h"

ros::ServiceClient spline_gen_cli;

bool compX(const std::vector<double>& a, const std::vector<double>& b)
{
    return a[0] < b[0];
}

bool genPath(base_trajectory_msgs::PointOrdering::Request& req, base_trajectory_msgs::PointOrdering::Response& res)
{
    /*
    const size_t points_num = req.points.size();
    const size_t constraint_num = req.constraints.size();

    // Store all points as a vector of vectors 
    std::vector<std::vector<double>> points;

    for(size_t i = 0; i < points_num; i++)
    {
        std::vector<double> v{i, req.points[i].positions[0], req.points[i].positions[1], req.points[i].positions[2]}; // Storing points in the form {ID, x, y, z} where ID is a unique number for each point for help with iterating all permutations
        points.push_back(v);
    }

    // We are going to iterate all possible permuations of all but the first point (first point is the starting point) 
    std::sort(points.begin()+1, points.end()-1, compX);

    do
    {
        // Make request to spline server
        base_trajectory_msgs::GenerateSpline spline_gen_srv;

        for(auto p : points)
            ROS_INFO_STREAM(p[1] << ' ' << p[2] << ' ' << p[3] << ' ');

        spline_gen_srv.request.constraints.resize(constraint_num);

        for (size_t i = 0; i < constraint_num; i++)
        {
            spline_gen_srv.request.constraints[i].corner1.x = req.constraints[i].corner1.x;
            spline_gen_srv.request.constraints[i].corner2.x = req.constraints[i].corner2.x;
            spline_gen_srv.request.constraints[i].corner1.y = req.constraints[i].corner1.y;
            spline_gen_srv.request.constraints[i].corner2.y = req.constraints[i].corner2.y;
            spline_gen_srv.request.constraints[i].max_accel = (req.constraints[i].max_accel < 0 ? std::numeric_limits<double>::max() : req.constraints[i].max_accel);
            spline_gen_srv.request.constraints[i].max_decel = (req.constraints[i].max_decel < 0 ? std::numeric_limits<double>::max() : req.constraints[i].max_decel);
            spline_gen_srv.request.constraints[i].max_vel = (req.constraints[i].max_vel <= 0 ? std::numeric_limits<double>::max() : req.constraints[i].max_vel);
            spline_gen_srv.request.constraints[i].max_cent_accel = (req.constraints[i].max_cent_accel <= 0 ? std::numeric_limits<double>::max() : req.constraints[i].max_cent_accel);
            spline_gen_srv.request.constraints[i].path_limit_distance = (req.constraints[i].path_limit_distance <= 0 ? std::numeric_limits<double>::max() : req.constraints[i].path_limit_distance);
        }
        
        spline_gen_srv.request.points.resize(points_num);
        for (size_t i = 0; i < points_num; i++)
        {
            spline_gen_srv.request.points[i].positions.resize(3);
            spline_gen_srv.request.points[i].positions[0] = points[i][1];
            spline_gen_srv.request.points[i].positions[1] = points[i][2];
            spline_gen_srv.request.points[i].positions[2] = points[i][3];
        }

        if (!spline_gen_cli.call(spline_gen_srv))
			ROS_ERROR_STREAM("Can't call spline gen service in path_follower_server");
		
        // Check the length of path
        const size_t num_waypoints = spline_gen_srv.response.path.poses.size();

        ROS_INFO_STREAM("Path length: " << spline_gen_srv.response.path.poses[num_waypoints-1].header.stamp - ros::Time::now());

		//debug
		//for (size_t i = 0; i < spline_gen_srv.response.end_points.size(); i++)
		//		ROS_INFO_STREAM("end point at " << i << " is " << spline_gen_srv.response.end_points[i]);
        
    } while(std::next_permutation(points.begin()+1, points.end()-1, compX)); // Iterate to the next permuation 
    */
    base_trajectory_msgs::GenerateSpline spline_gen_srv;
    const size_t objects_num = req.detection.objects.size();

    std::vector<std::vector<double>> points;
    points.push_back({0,0,0});

    for (size_t i = 0; i < objects_num; i++)
    {
        if(req.detection.objects[i].id == "power_cell")
        {
            std::vector<double> v{req.detection.objects[i].location.x, req.detection.objects[i].location.y, req.detection.objects[i].location.z};
            points.push_back(v);
        }
    }

    std::vector<double> v{8.382, points[points.size()-1], 0};
    points.push_back(v);

    size_t points_num = points.size();

    spline_gen_srv.request.points.resize(points_num);
    for (size_t i = 0; i < points_num; i++)
    {
        spline_gen_srv.request.points[i].positions.resize(3);
        spline_gen_srv.request.points[i].positions[0] = points[i][0];
        spline_gen_srv.request.points[i].positions[1] = points[i][1];
        if(i > 0) 
            spline_gen_srv.request.points[i].positions[2] = math::atan2(points[i][1]-points[i-1], points[i][0]-points[i-1][0]); // right triangle math to calculate angle 
        else
            spline_gen_srv.request.points[i].positions[2] = 0;
    }
    
    spline_gen_srv.request.optimize_final_velocity = false; // flag for optimized velocity
    // pass in frame id

    if (!spline_gen_cli.call(spline_gen_srv))
	    ROS_ERROR_STREAM("Can't call spline gen service in path_follower_server");

    res = spline_gen_cli.response;

    return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "galactic_path_server");
	ros::NodeHandle nh;

    ros::ServiceServer svc = nh.advertiseService("galactic_path_gen", genPath);

    spline_gen_cli = nh.serviceClient<base_trajectory_msgs::GenerateSpline>("/base_trajectory/spline_gen");

    ros::spin();
    return 0;

}
