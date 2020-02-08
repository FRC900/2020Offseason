#include "world_model.hpp"
#include <cmath>
#include <random>
#include <utility>
#include <algorithm>

#include <iostream>
#include <string>

#include <ros/ros.h>


WorldModel::WorldModel(std::vector<std::pair<double, double> > beacons,
                       double x_min, double x_max, double y_min, double y_max) {
  beacons_ = beacons;
  boundaries_[0] = x_min;
  boundaries_[1] = x_max;
  boundaries_[2] = y_min;
  boundaries_[3] = y_max;
}

std::vector<double> WorldModel::get_boundaries() {
  std::vector<double> res;
  for (size_t i = 0; i < 4; i++) {
    res.push_back(boundaries_[i]);
  }
  return res;
}

//checks if a given particle is within the defined boundaries
bool WorldModel::is_in_world(const Particle& p) const {
  if(p.x < boundaries_[0] || p.x > boundaries_[1]){
    return false;
  }
  if(p.y < boundaries_[2] || p.y > boundaries_[3]){
    return false;
  }
  return true;
}

//moves a given particle to the nearest position that is within the defined boundaries
void WorldModel::constrain_to_world(Particle& p) const {
  p.x = std::min(boundaries_[1], std::max(boundaries_[0], p.x));
  p.y = std::min(boundaries_[3], std::max(boundaries_[2], p.y));
}

//computes distances from a position a set of beacon positions
std::vector<double> WorldModel::distances(const std::pair<double, double> m,
                                          const std::vector<std::pair<double, double> > rel) const {
  std::vector<double> res;
  res.reserve(beacons_.size());
  for (std::pair<double, double> b : rel) {
    res.push_back(hypot(m.first - b.first, m.second - b.second));
  }
  return res;
}

//gets the coordinates of all the beacons relative to a give particle
std::vector<std::pair<double, double> > WorldModel::particle_relative(const Particle& p) const {
  std::vector<std::pair<double, double> > res;
  for (std::pair<double, double> b : beacons_) {

    double x = b.first - p.x;
    double y = b.second - p.y;
    double r = hypot(x, y);
    double theta = atan2(y, x);
    theta -= p.rot;
    x = r * cos(theta);
    y = r * sin(theta);

    res.push_back(std::make_pair(x, y));
  }
  return res;
}

//Uses hungarian algorithm to pair particle relatiev beacons and robot relative beacons and returns the total error (sum of distance errors from particle to robot beacons)
double WorldModel::total_distance(const Particle& p, const std::vector<std::pair<double, double> >& m) {
  std::vector<int> assignment;
  assignment.reserve(m.size());
  std::vector<std::vector<double> > dists;
  dists.reserve(m.size());
  std::vector<std::pair<double, double> > rel = particle_relative(p);
  for (std::pair<double, double> b : m) {
    dists.push_back(distances(b, rel));
  }
  solver_.Solve(dists, assignment);
  double res = 0;
  for (size_t i = 0; i < assignment.size(); i++) {
    res += dists[i][assignment[i]];
  }
  return res;
}
