#ifndef PARTICLE_HEADER
#define PARTICLE_HEADER
#include <cmath>
#include <ostream>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/Pose.h>
// #define CHECK_NAN
struct Particle {
  double x_;
  double y_;
  double rot_;
  double weight_;

  Particle(double x, double y, double rot): x_(x), y_(y), rot_(rot), weight_(1) {}
  Particle() : x_(0), y_(0), rot_(0), weight_(1) {}
  bool isValid(void) const
  {
	  const double sum = x_ + y_ + rot_ + weight_;
	  return !std::isnan(sum) && !std::isinf(sum);
  }

  geometry_msgs::Pose toPose() {
    geometry_msgs::Pose pose;
    pose.position.x = x_;
    pose.position.y = y_;
    pose.position.z = 0;

    tf2::Quaternion q;
    q.setRPY(0, 0, rot_);
    pose.orientation.x = q.getX();
    pose.orientation.y = q.getY();
    pose.orientation.z = q.getZ();
    pose.orientation.w = q.getW();
    return pose;
  }
};

std::ostream& operator<<(std::ostream& os, const Particle &p);

#endif
