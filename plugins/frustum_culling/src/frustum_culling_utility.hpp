#pragma once

#include <cmath>
#include <cloe/component/frustum.hpp>  // for frustum
#include <cloe/component/object.hpp>   // for Object

struct Point {
  double x{0.0};
  double y{0.0};
};

// does not look at fov_z, so only looks in x-y plane
// p0 to p3 are the points counter clock-wise, starting from the bottom-left
//
//
//   clip_far  p3   ----------- p2
//                  \         /
//  clip_near   p0   \_______/ p1
//
std::vector<Point> calculate_corner_points_in_frustum_coordinates(const cloe::Frustum& frustum) {
    Point p0{0.0, 0.0};
    Point p1{p0};
    Point p2{p0};
    Point p3{p0};

    std::vector<Point> retPoints;

    if (frustum.clip_near > 0.0)
    {
        p0.x = frustum.clip_near;
        p1.x = frustum.clip_near;
        
        p0.y = frustum.clip_near * std::tan(frustum.fov_h/2.0);
        p1.y = -frustum.clip_near * std::tan(frustum.fov_h/2.0);

        retPoints.push_back(p0);
        retPoints.push_back(p1);
    }
    else {
        retPoints.push_back(p0);
    }

    p2.x = frustum.clip_far;
    p3.x = frustum.clip_far;

    p2.y = -frustum.clip_far * std::tan(frustum.fov_h/2.0);
    p3.y = frustum.clip_far * std::tan(frustum.fov_h/2.0);

    retPoints.push_back(p2);
    retPoints.push_back(p3);

    return retPoints;
}

// rotates point from one coordinate system to another
// angle is the angle between the x axis of the new coordinate system compared to the x axis to the original 
Point rotate_point(const Point& point, double angle)
{
  Point p{0.0, 0.0};
  p.x = std::cos(angle) * point.x - std::sin(angle) * point.y;
  p.y = std::sin(angle) * point.x + std::cos(angle) * point.y;
  return p;
}

bool is_object_inside_polygon(const cloe::Object& object, std::vector<Point> points) {
  bool retVal = false;

  // points of corners
  if (points.size() == 0) {
    return false;
  }

  auto current_corner = points[0];
  int num_edges = points.size();
  double x = object.pose.translation().x();
  double y = object.pose.translation().y();
  for (int i = 0; i < points.size(); i++) {
    auto next_corner = points[(i + 1) % num_edges];

    if (x > std::min(current_corner.x, next_corner.x)) {
      if (x <= std::max(current_corner.x, next_corner.x)) {
        if (y <= std::max(current_corner.y, next_corner.y)) {
          double y_intersection = (x - current_corner.x) * (next_corner.y - current_corner.y) /
                                      (next_corner.x - current_corner.x) +
                                  current_corner.y;
          if (current_corner.y == next_corner.y || y <= y_intersection) {
            retVal = !retVal;
          }
        }
      }
    }
    current_corner = next_corner;
  }
  return retVal;
}