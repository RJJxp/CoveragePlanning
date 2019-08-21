#ifndef _BOW_SHAPED_PLANNER_H_
#define _BOW_SHAPED_PLANNER_H_

#include <iostream>
// #include <glog/logging.h>
#include <ros_msgs/Trajectory.h>
#include <ros_msgs/Odometry.h>
#include <ros_msgs/Vector2.h>
#include <ros_msgs/PoseWithVelocity.h>

#include <opencv2/opencv.hpp>

// generated a bow-shaped path with the given convex polygon
// input `odometry` is not used, only to uniform the interface in tergeo
// `sweeping_area` is the given convex polygon
// `traj` is the path planed
// Example:
//      BowShapedPlanner* bsp = new BowShapePlanner();
//      bsp->coveragePlan();
class BowShapedPlanner {
public:
    BowShapedPlanner();
    ~BowShapedPlanner();
    
    // bow shaped coverage plan by rjp
    bool coveragePlan(const ros_msgs::Odometry& odometry,
                      const std::vector<ros_msgs::Vector2>& sweeping_area,
                      std::vector<ros_msgs::Trajectory>& multi_traj);
private:
    // calculate the private variable `_rotate_distance`
    bool getRotateAngle(const std::vector<ros_msgs::Vector2>& in_sweeping_area, double& rotate_angle);
    // rotate the sweeping_area
    // rotate the longest edge to pi/2
    bool rotateSweepArea(const std::vector<ros_msgs::Vector2>& in_sweeping_area,
                         double in_angle,
                         std::vector<ros_msgs::Vector2>& out_sweeping_area);
    // get the minx, miny, maxx, maxy of the given polygon
    bool getBoundOfSweepArea(const std::vector<ros_msgs::Vector2> in_sweeping_area,
                             double& min_x, double& min_y, double& max_x, double& max_y);
    // get the origin path which contains turning points only
    // turning points include all the start points and end points of the long bow-shape line                     
    bool getTurnPointOfBowShape(const std::vector<ros_msgs::Vector2>& in_sweeping_area,
                                std::vector<std::pair<ros_msgs::Vector2, ros_msgs::Vector2> >& out_ori_path);
    // add extra points between the start point and the end point for long and short bow-shape lines 
    bool getStraightPointOfBowShape(const std::vector<std::pair<ros_msgs::Vector2, ros_msgs::Vector2> >& in_ori_path,
                                    double in_long_dist, double in_short_dist,
                                    std::vector<ros_msgs::Vector2>& out_result_path);
    // get the result point with given x coordinate
    // if the result point's x range is between in_pt1 and in_pt2, return true, otherwise return false
    bool getPointFromX(const ros_msgs::Vector2& in_pt1, const ros_msgs::Vector2& in_pt2, 
                       double in_x, ros_msgs::Vector2& out_pt);
    // Following are two functions mainly used in getStriaghtPointOfBowShape()
    // get the striaght points given the long bow-shape line in the form of std::pair
    bool getLongLinePoint(const std::pair<ros_msgs::Vector2, ros_msgs::Vector2>& in_pt_pair,
                          double long_dist,
                          std::vector<ros_msgs::Vector2>& out_part_traject);
    // given the short bow-shape line in the form first pair.second and the second pair.first 
    bool getShortLinePoint(const std::pair<ros_msgs::Vector2, ros_msgs::Vector2>& in_pt_pair_01,
                           const std::pair<ros_msgs::Vector2, ros_msgs::Vector2>& in_pt_pair_02,
                           double short_dist,
                           std::vector<ros_msgs::Vector2>& out_part_traject);
    // calculate the angle between the X axis positive direction and vector(pt1, pt2)
    // TODO: the getAngle function is not good with input variable
    double getAngle(const ros_msgs::Vector2& pt1, const ros_msgs::Vector2& pt2);
    // plan for a convex polygon
    bool plan4ConvexPolygon(const std::vector<ros_msgs::Vector2>& in_sweeping_area, ros_msgs::Trajectory& out_traj);

private:
    double _rotate_angle;       // pi/2 - angle of the shortest line of the sweeping area

    // param to control the bow shape
    // TODO: get value of them from config file including the sweeping area
    // TODO: temporarily ignore the transform between 2 coordinate systems of map and world
    // TODO: ambiguous distance unit: pixel or m? right now they are pixel
    double _offset_distance;        // the distance offset inside the sweeping area to avoid collision, more like a buffer distance
    double _space_distance;         // the distance between the 2 bow-shape long lines
    double _ref_point_long_dist;    // in the long bow-shape line with only start point and end point
                                    // add an extra ref_point every _ref_point_long_dist for the following planning
                                    // details in function `planning_plugins/raccoon_planning/reference_line::autoMarkReferenceLine`
    double _ref_point_short_dist;    // same as _ref_point_long_dist, the distance between short line

};


#endif