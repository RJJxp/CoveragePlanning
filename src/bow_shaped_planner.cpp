#include "bow_shaped_planner.h"

BowShapedPlanner::BowShapedPlanner(){
    _offset_distance = 5;
    _space_distance = 8;
    _ref_point_long_dist = 60;
    _ref_point_short_dist = 10;
}

BowShapedPlanner::~BowShapedPlanner(){
}


bool BowShapedPlanner::coveragePlan(const ros_msgs::Odometry& odometry,
                                    const std::vector<ros_msgs::Vector2>& sweeping_area,
                                    ros_msgs::Trajectory& traj) {
    if(1) {
        
        std::vector<ros_msgs::Vector2> my_sweeping_area = {};
        for (int i = 0; i < sweeping_area.size(); ++i) {
            my_sweeping_area.push_back(sweeping_area[i]);
        }
        my_sweeping_area.push_back(sweeping_area[0]);
        // use varible `my_sweeping_area` to replace `sweeping_area`
        // ATTENTION: `my_sweeping_area` has the same first and last element
        // LOG(INFO) << "finish copying sweeping area";
        std::cout << "finish copying sweeping area"<< std::endl;

        if (!getRotateAngle(my_sweeping_area)) {
            // LOG(INFO) << "get rotate angle failed";
            std::cout << "get rotate angle failed" << std::endl;
            return false;
        }

        std::vector<ros_msgs::Vector2> my_sweeping_area_rotate = {};
        if(!rotateSweepArea(my_sweeping_area, _rotate_angle, my_sweeping_area_rotate)) {
            // LOG(INFO) << "rotate sweep area failed";
            std::cout << "rotate sweep area failed" << std::endl;
            return false;
        }
        
        std::vector<std::pair<ros_msgs::Vector2, ros_msgs::Vector2> > ori_path;
        if (!getTurnPointOfBowShape(my_sweeping_area_rotate, ori_path)) {
            // LOG(INFO) << "get turn point of bow-shape failed";
            std::cout << "get turn point of bow-shape failed" << std::endl;
            return false;
        }

        std::vector<ros_msgs::Vector2> result_path;
        if (!getStraightPointOfBowShape(ori_path, _ref_point_long_dist, _ref_point_short_dist, result_path)) {
            // LOG(INFO) << "get straight line points failed";
            std::cout << "get straight line points failed" << std::endl;
            return false;
        }

        std::vector<ros_msgs::Vector2> result_path_rotate;
        if (!rotateSweepArea(result_path, -_rotate_angle, result_path_rotate)) {
            // LOG(INFO) << "get result path rotate failed";
            std::cout << "get result path rotate failed" << std::endl;
            return false;
        }

        ros_msgs::PoseWithVelocity tmp_pose;
        for (int i = 0; i < result_path_rotate.size(); ++i) {
            // std::cout << "x " << result_path_rotate[i].x << std::endl << "y " << result_path_rotate[i].y << std::endl;
            tmp_pose.position.x = result_path_rotate[i].x;
            tmp_pose.position.y = result_path_rotate[i].y;
            tmp_pose.position.z = 0;
            traj.poses.push_back(tmp_pose);
        }
        std::cout << "coverage plan succeed" << std::endl;
        return true;
    } else {

        return false;
    }

}

bool BowShapedPlanner::getRotateAngle(const std::vector<ros_msgs::Vector2>& in_sweeping_area) {
    // make sure sweeping area is a polygon
    if (in_sweeping_area.size() < 3) {
        // LOG(INFO) << "the sweeping area is not a polygon in `getRotationAngle`";
        std::cout << "the sweeping area is not a polygon in `getRotationAngle`" << std::endl;
        return false;
    }

    double dist, del_x, del_y;
    for (int i = 0; i < in_sweeping_area.size(); ++i) {
        del_x = in_sweeping_area[i + 1].x - in_sweeping_area[i].x;
        del_y = in_sweeping_area[i + 1].y - in_sweeping_area[i].y;
        if (i == 0) {   // the first point
            dist = del_x * del_x + del_y * del_y;
        } else {    // the other points
            if (dist < del_x * del_x + del_y * del_y) {
                dist = del_x * del_x + del_y * del_y;
                _rotate_angle = M_PI / 2 - atan2f(del_y, del_x);
            }
        }
    }
    std::cout << "get rotate angle succeed, the angle is " << _rotate_angle << " rad" << std::endl; 
    return true;
}

bool BowShapedPlanner::rotateSweepArea(const std::vector<ros_msgs::Vector2>& in_sweeping_area,
                                       double in_angle,
                                       std::vector<ros_msgs::Vector2>& out_sweeping_area) {
    if (in_sweeping_area.size() < 4) {
        // LOG(INFO) << "the sweeping area is not a polygon in `rotateSweepArea`";
        std::cout << "the sweeping area is not a polygon in `rotateSweepArea`" << std::endl;
        return false;
    }

    out_sweeping_area.clear();
    ros_msgs::Vector2 tmp_vec;
    // rotate counterclockwise
    for (int i = 0;i < in_sweeping_area.size(); ++i) {
        tmp_vec.x = cos(in_angle) * in_sweeping_area[i].x - sin(in_angle) * in_sweeping_area[i].y;
        tmp_vec.y = sin(in_angle) * in_sweeping_area[i].x + cos(in_angle) * in_sweeping_area[i].y;
        out_sweeping_area.push_back(tmp_vec);
    }
    std::cout << "finished rotate sweep area" << std::endl;
    return true;
}

bool BowShapedPlanner::getBoundOfSweepArea(const std::vector<ros_msgs::Vector2> in_sweeping_area,
                                           double& min_x, double& min_y, double& max_x, double& max_y) {
    if (in_sweeping_area.size() < 4) {
        // LOG(INFO) << "the sweeping area is not a polygon in `getBoundOfSweepArea`";
        std::cout << "the sweeping area is not a polygon in `getBoundOfSweepArea`" << std::endl;
        return false;
    }

    // use iter and std function to calculate the min and max
    // auto iter = std::min_element(in_sweeping_area.begin(), in_sweeping_area.end(), 
    //                         [](const ros_msgs::Vector2 v1, const ros_msgs::Vector2 v2){
    //                             return v1.x < v2.x;
    //                         });
    // min_x = *iter;
    // iter = std::min_element(in_sweeping_area.begin(), in_sweeping_area.end(), 
    //                         [](const ros_msgs::Vector2 v1, const ros_msgs::Vector2 v2){
    //                             return v1.y < v2.y;
    //                         });
    // min_y = *iter;
    // iter = std::max_element(in_sweeping_area.begin(), in_sweeping_area.end(), 
    //                         [](const ros_msgs::Vector2 v1, const ros_msgs::Vector2 v2){
    //                             return v1.x < v2.x;
    //                         });
    // max_x = *iter;
    // iter = std::max_element(in_sweeping_area.begin(), in_sweeping_area.end(), 
    //                         [](const ros_msgs::Vector2 v1, const ros_msgs::Vector2 v2){
    //                             return v1.y < v2.y;
    //                         });
    // max_y = *iter;

    for (int i = 0; i < in_sweeping_area.size(); ++i) {
        if (i == 0) {
            min_x = in_sweeping_area[i].x;
            max_x = in_sweeping_area[i].x;
            min_y = in_sweeping_area[i].y;
            max_y = in_sweeping_area[i].y;
        } else {
            min_x = in_sweeping_area[i].x < min_x ? in_sweeping_area[i].x : min_x;
            min_y = in_sweeping_area[i].y < min_y ? in_sweeping_area[i].y : min_y;
            max_x = max_x < in_sweeping_area[i].x ? in_sweeping_area[i].x : max_x;
            max_y = max_y < in_sweeping_area[i].y ? in_sweeping_area[i].y : max_y;
        }
        
    }
    std::cout << "get bound func is finished" << std::endl;
    std::cout << "minx:\t" << min_x << "\t max_x \t"  << max_x << std::endl;
    std::cout << "miny:\t" << min_y << "\t max_y \t"  << max_y << std::endl;
    return true;
}

// use a couple of cut-lines which are parallel with y axis to intersect the polygon 
// the part of cut-line inside the polygon become the long bow-shape line
// the 2 intersections become the start point and end point, which I call turn point in general 
// then arrange them in order
bool BowShapedPlanner::getTurnPointOfBowShape(const std::vector<ros_msgs::Vector2>& in_sweeping_area,
                                              std::vector<std::pair<ros_msgs::Vector2, ros_msgs::Vector2> >& out_ori_path) {
    if (in_sweeping_area.size() < 4) {
        // LOG(INFO) << "sweeping area is not a polygon in getTurnPoint func";
        std::cout << "sweeping area is not a polygon in getTurnPoint func" << std::endl;
        return false;
    }
    
    double min_x, min_y, max_x, max_y;
    if (!getBoundOfSweepArea(in_sweeping_area, min_x, min_y, max_x, max_y)) {
        // LOG(INFO) << "get bound of sweep area failed";
        std::cout << "get bound of sweep area failed" << std::endl;
        return false;
    }

    std::pair<ros_msgs::Vector2, ros_msgs::Vector2> tmp_pair;
    std::vector<ros_msgs::Vector2> tmp_vec; // every time enter a loop, clear this vector
    ros_msgs::Vector2 tmp_pt;
    bool order_flag = true;
    double cutline = min_x + _offset_distance;
    
    while (cutline + _space_distance < max_x - _offset_distance) {
        for (int i = 0; i < in_sweeping_area.size(); ++i) {
            if (getPointFromX(in_sweeping_area[i], in_sweeping_area[i + 1], cutline, tmp_pt)) {
                tmp_vec.push_back(tmp_pt);
            } else {
                // LOG(INFO) << "get point from " << cutline << "  GET A PROBLEM";
                // std::cout << "get point from " << cutline << "  GET A PROBLEM" << std::endl;
                // return false;
            }
        }
        // arrange in order
        // guarantee the out_ori_path[i].second's next point is out_ori_path[i + 1].first
        if (tmp_vec.size() < 2) {
            std::cout << "not enough points from cutline " << cutline << std::endl;
            return false;
        }
        if (order_flag) {
            tmp_pair.first = tmp_vec[0];
            tmp_pair.second = tmp_vec[1];
            order_flag = false;
        } else {
            tmp_pair.first = tmp_vec[1];
            tmp_pair.second = tmp_vec[0];
            order_flag = true;
        }
        tmp_vec.clear();
        out_ori_path.push_back(tmp_pair);
        cutline += _space_distance;
    }
    std::cout << "the turn path length is " << out_ori_path.size() << std::endl;
    std::cout << "get turn point func succeed" << std::endl;

    // for (int i = 0; i < out_ori_path.size(); ++i) {
    //     std::cout << "fir.x " << out_ori_path[i].first.x << " fir.y " << out_ori_path[i].first.y << std::endl;
    //     std::cout << "sec.x " << out_ori_path[i].second.x << " sec.y " << out_ori_path[i].second.y << std::endl;  
    // }

    // for visualization debug
    cv::Mat my_panel(1000, 1000, CV_8UC3, cv::Scalar(0, 0, 0));
    for (int i = 0; i < out_ori_path.size(); ++i) {
        cv::Point2f tmp_cv_pt;
        tmp_cv_pt.x = cos(_rotate_angle) * out_ori_path[i].first.x + sin(_rotate_angle) * out_ori_path[i].first.y;
        tmp_cv_pt.y = sin(-_rotate_angle) * out_ori_path[i].first.x + cos(_rotate_angle) * out_ori_path[i].first.y;
        cv::circle(my_panel, tmp_cv_pt, 3, cv::Scalar(255, 255, 0), 1, CV_FILLED);
        tmp_cv_pt.x = cos(_rotate_angle) * out_ori_path[i].second.x + sin(_rotate_angle) * out_ori_path[i].second.y;
        tmp_cv_pt.y = sin(-_rotate_angle) * out_ori_path[i].second.x + cos(_rotate_angle) * out_ori_path[i].second.y;
        cv::circle(my_panel, tmp_cv_pt, 3, cv::Scalar(255, 255, 0), 1, CV_FILLED);
    }
    cv::imshow("turn points", my_panel);
    cv::waitKey(0);

    return true;
}

bool BowShapedPlanner::getPointFromX(const ros_msgs::Vector2& in_pt1, const ros_msgs::Vector2& in_pt2, 
                                     double in_x, ros_msgs::Vector2& out_pt) {
    if ((in_pt1.x - in_x) * (in_pt2.x - in_x) > 0) return false;

    // the line of in_pt1 and in_pt2 is parallel to y axis
    if (in_pt2.x == in_pt1.x) return false;

    double scale = (in_x - in_pt1.x) / (in_pt2.x - in_pt1.x);
    out_pt.x = in_x;
    out_pt.y = in_pt1.y + scale * (in_pt2.y - in_pt1.y);
    return true;
}

// bow-shape long line is parallel with y axis
// bow-shape short line is parallel with x axis
bool BowShapedPlanner::getStraightPointOfBowShape(const std::vector<std::pair<ros_msgs::Vector2, ros_msgs::Vector2> >& in_ori_path,
                                                  double in_long_dist, double in_short_dist,
                                                  std::vector<ros_msgs::Vector2>& out_result_path) {
    std::vector<ros_msgs::Vector2> tmp_path;
    for (int i = 0; i < in_ori_path.size() - 1; ++i) {
        if (!getLongLinePoint(in_ori_path[i], in_long_dist, tmp_path)) {
            // LOG(INFO) << i << ": get long line point failed";     
            std::cout << ": get long line point failed" << std::endl;
            return false;
        }
        out_result_path.insert(out_result_path.end(), tmp_path.begin(), tmp_path.end());
        if (!getShortLinePoint(in_ori_path[i], in_ori_path[i + 1], in_short_dist, tmp_path)) {
            // LOG(INFO) << i << ": get short line point failed";
            std::cout << ": get short line point failed" << std::endl;
            return false;
        }
        out_result_path.insert(out_result_path.end(), tmp_path.begin(), tmp_path.end());
    }

    if (!getLongLinePoint(in_ori_path[in_ori_path.size() - 1], in_long_dist, tmp_path)) {
        // LOG(INFO) << "the last get long line point failed";
        std::cout << "the last get long line point failed" << std::endl;
        return false;
    }
    
    out_result_path.insert(out_result_path.end(), tmp_path.begin(), tmp_path.end());
    out_result_path.push_back(in_ori_path[in_ori_path.size() - 1].second);
    return true;
}

// the straight points in long bow-shape line range from [start_pt, end_pt)
bool BowShapedPlanner::getLongLinePoint(const std::pair<ros_msgs::Vector2, ros_msgs::Vector2>& in_pt_pair,
                                        double long_dist,
                                        std::vector<ros_msgs::Vector2>& out_part_traject) {
    out_part_traject.clear();

    if (in_pt_pair.first.x != in_pt_pair.second.x) {
        // LOG(INFO) << "the given pt pair's X is not equal";
        std::cout << "the given pt pair's X is not equal" << std::endl;
        return false;
    }

    double length = in_pt_pair.second.y - in_pt_pair.first.y;
    double length_flag = length;
    if (length < 0) {
        long_dist = -long_dist;
    }

    ros_msgs::Vector2 tmp_pt = in_pt_pair.first;
    out_part_traject.push_back(tmp_pt);
    length_flag = length_flag - long_dist;
    while (length * length_flag > 0) {
        length_flag = length_flag - long_dist;
        tmp_pt.y += long_dist;
        out_part_traject.push_back(tmp_pt);        
    }
    return true;
}

// the straight points in short bow-shape line range from [pair_01.second, pair_02.first)
// move short dist every time along the vector(pair_01.second, pair_02.first)
bool BowShapedPlanner::getShortLinePoint(const std::pair<ros_msgs::Vector2, ros_msgs::Vector2>& in_pt_pair_01,
                                         const std::pair<ros_msgs::Vector2, ros_msgs::Vector2>& in_pt_pair_02,
                                         double short_dist,
                                         std::vector<ros_msgs::Vector2>& out_part_traject) {
    out_part_traject.clear();
    double length = in_pt_pair_02.first.x - in_pt_pair_01.second.x;
    double slope = (in_pt_pair_02.first.y - in_pt_pair_01.second.y) / length;
    double angle = getAngle(in_pt_pair_01.second, in_pt_pair_02.first);
    double length_flag = length;
    double short_dist_copy = short_dist;
    if (length < 0) {
        short_dist = -short_dist;
    }
    
    ros_msgs::Vector2 tmp_pt = in_pt_pair_01.second;
    out_part_traject.push_back(tmp_pt);
    length_flag = length_flag - short_dist * fabs(cos(angle));
    while (length * length_flag > 0) {
        length_flag = length_flag - short_dist;
        tmp_pt.x += short_dist_copy * cos(angle);
        tmp_pt.y += short_dist_copy * sin(angle);
        out_part_traject.push_back(tmp_pt);
    }
    return true;
}

double BowShapedPlanner::getAngle(const ros_msgs::Vector2& pt1, const ros_msgs::Vector2& pt2) {
    // parallel with axis y
    if (pt1.x == pt2.x) {
        if (pt1.y < pt2.y) {
            return M_PI_2;
        } else {
            return -M_PI_2;
        }
    }
    // function `atan2` ranges from (-pi/2, pi/2)
    // which indicates the pt2 is in the right of pt1 by default
    double angle_atan = atan2(pt2.y - pt1.y, pt2.x - pt1.x);
    // if the pt2 is in the left of pt1
    if (pt2.x < pt1.x) {
        angle_atan += M_PI;
    } 
    return angle_atan;
}