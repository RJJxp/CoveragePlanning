#include "bow_shaped_planner.h"
#include "decomposition.h"

BowShapedPlanner::BowShapedPlanner(){
    _offset_distance = 5;
    _space_distance = 10;
    _ref_point_long_dist = 20;
    _ref_point_short_dist = 20;
}

BowShapedPlanner::~BowShapedPlanner(){
}


bool BowShapedPlanner::coveragePlan(const ros_msgs::Odometry& odometry,
                                    const std::vector<RjpPoint>& sweeping_area,
                                    std::vector<RjpTrajectory>& multi_traj) {
    multi_traj.clear();

    PolygonDecomposition pdc;
    std::vector<std::vector<RjpPoint> > split_sweeping_area;
    pdc.decomposePolygon(sweeping_area, split_sweeping_area);
    // debug
    cv::Mat my_panel(1000, 1000, CV_8UC3, cv::Scalar(40, 0, 0));
    for (int i = 0; i < split_sweeping_area.size(); ++i) {
        double x, y;
        for (int j = 0; j < split_sweeping_area[i].size() - 1; ++j) {
            x = split_sweeping_area[i][j].x;
            y = split_sweeping_area[i][j].y;
            cv::Point2f p1(x, y);
            x = split_sweeping_area[i][j + 1].x;
            y = split_sweeping_area[i][j + 1].y;
            cv::Point2f p2(x, y);
            cv::line(my_panel, p1, p2, cv::Scalar(0, 200, 0), 2);
        }
        x = split_sweeping_area[i][split_sweeping_area[i].size() - 1].x;
        y = split_sweeping_area[i][split_sweeping_area[i].size() - 1].y;
        cv::Point2f p1(x, y);
        x = split_sweeping_area[i][0].x;
        y = split_sweeping_area[i][0].y;
        cv::Point2f p2(x, y);
        cv::line(my_panel, p1, p2, cv::Scalar(0, 200, 0), 2);
    }
    cv::imshow("split_polygons", my_panel);
    cv::waitKey(0);
    
    std::cout << "****************  decompose finished  ****************" << std::endl;
    for (int i = 0; i < split_sweeping_area.size(); ++i) {
        RjpTrajectory sub_traj;
        plan4ConvexPolygon(split_sweeping_area[i], sub_traj);
        multi_traj.push_back(sub_traj);
    }
    return true;

}

bool BowShapedPlanner::plan4ConvexPolygon(const std::vector<RjpPoint>& in_sweeping_area,
                                          RjpTrajectory& out_traj) {
    std::vector<RjpPoint> my_sweeping_area = {};
    for (int i = 0; i < in_sweeping_area.size(); ++i) {
        my_sweeping_area.push_back(in_sweeping_area[i]);
    }

    my_sweeping_area.push_back(in_sweeping_area[0]);
    // use varible `my_sweeping_area` to replace `sweeping_area`
    // ATTENTION: `my_sweeping_area` has the same first and last element
    std::cout << "finish copying sweeping area"<< std::endl;

    double rotate_angle = 0.0;
    if (!getRotateAngle(my_sweeping_area, rotate_angle)) {
        std::cout << "get rotate angle failed" << std::endl;
        return false;
    }
    // for debug 
    _rotate_angle = rotate_angle;

    std::vector<RjpPoint> my_sweeping_area_rotate = {};
    if(!rotateSweepArea(my_sweeping_area, rotate_angle, my_sweeping_area_rotate)) {
        std::cout << "rotate sweep area failed" << std::endl;
        return false;
    }
    
    std::vector<std::pair<RjpPoint, RjpPoint> > ori_path;
    if (!getTurnPointOfBowShape(my_sweeping_area_rotate, ori_path)) {
        std::cout << "get turn point of bow-shape failed" << std::endl;
        return false;
    }

    std::vector<RjpPoint> result_path;
    if (!getStraightPointOfBowShape(ori_path, _ref_point_long_dist, _ref_point_short_dist, result_path)) {
        std::cout << "get straight line points failed" << std::endl;
        return false;
    }

    std::vector<RjpPoint> result_path_rotate;
    if (!rotateSweepArea(result_path, -rotate_angle, result_path_rotate)) {
        std::cout << "get result path rotate failed" << std::endl;
        return false;
    }

    RjpPoint tmp_pose;
    for (int i = 0; i < result_path_rotate.size(); ++i) {
        // std::cout << "x " << result_path_rotate[i].x << std::endl << "y " << result_path_rotate[i].y << std::endl;
        
        tmp_pose.x = result_path_rotate[i].x;
        tmp_pose.y = result_path_rotate[i].y;
        out_traj.pts.push_back(tmp_pose);
    }
    std::cout << "coverage plan succeed" << std::endl;
    return true;
}

bool BowShapedPlanner::getRotateAngle(const std::vector<RjpPoint>& in_sweeping_area,
                                      double& rotate_angle) {
    // make sure sweeping area is a polygon
    if (in_sweeping_area.size() < 3) {
        std::cout << "the sweeping area is not a polygon in `getRotationAngle`" << std::endl;
        return false;
    }

    double dist, del_x, del_y;
    for (int i = 0; i < in_sweeping_area.size() - 1; ++i) {
        del_x = in_sweeping_area[i + 1].x - in_sweeping_area[i].x;
        del_y = in_sweeping_area[i + 1].y - in_sweeping_area[i].y;
        if (i == 0) {   // the first point
            dist = del_x * del_x + del_y * del_y;
            rotate_angle = M_PI / 2 - atan2f(del_y, del_x);
        } else {    // the other points
            if (dist < del_x * del_x + del_y * del_y) {
                dist = del_x * del_x + del_y * del_y;
                rotate_angle = M_PI / 2 - atan2f(del_y, del_x);
            }
        }
        // std::cout << i << ": " << dist << std::endl;
    }
    std::cout << "get rotate angle succeed, the angle is " << rotate_angle << " rad" << std::endl; 
    return true;
}

bool BowShapedPlanner::rotateSweepArea(const std::vector<RjpPoint>& in_sweeping_area,
                                       double in_angle,
                                       std::vector<RjpPoint>& out_sweeping_area) {
    if (in_sweeping_area.size() < 4) {
        std::cout << "the sweeping area is not a polygon in `rotateSweepArea`" << std::endl;
        return false;
    }

    out_sweeping_area.clear();
    RjpPoint tmp_vec;
    // rotate counterclockwise
    for (int i = 0;i < in_sweeping_area.size(); ++i) {
        tmp_vec.x = cos(in_angle) * in_sweeping_area[i].x - sin(in_angle) * in_sweeping_area[i].y;
        tmp_vec.y = sin(in_angle) * in_sweeping_area[i].x + cos(in_angle) * in_sweeping_area[i].y;
        out_sweeping_area.push_back(tmp_vec);
    }
    std::cout << "finished rotate sweep area" << std::endl;
    return true;
}

bool BowShapedPlanner::getBoundOfSweepArea(const std::vector<RjpPoint> in_sweeping_area,
                                           double& min_x, double& min_y, double& max_x, double& max_y) {
    if (in_sweeping_area.size() < 4) {
        std::cout << "the sweeping area is not a polygon in `getBoundOfSweepArea`" << std::endl;
        return false;
    }

    // use iter and std function to calculate the min and max
    // auto iter = std::min_element(in_sweeping_area.begin(), in_sweeping_area.end(), 
    //                         [](const RjpPoint v1, const RjpPoint v2){
    //                             return v1.x < v2.x;
    //                         });
    // min_x = *iter;
    // iter = std::min_element(in_sweeping_area.begin(), in_sweeping_area.end(), 
    //                         [](const RjpPoint v1, const RjpPoint v2){
    //                             return v1.y < v2.y;
    //                         });
    // min_y = *iter;
    // iter = std::max_element(in_sweeping_area.begin(), in_sweeping_area.end(), 
    //                         [](const RjpPoint v1, const RjpPoint v2){
    //                             return v1.x < v2.x;
    //                         });
    // max_x = *iter;
    // iter = std::max_element(in_sweeping_area.begin(), in_sweeping_area.end(), 
    //                         [](const RjpPoint v1, const RjpPoint v2){
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
bool BowShapedPlanner::getTurnPointOfBowShape(const std::vector<RjpPoint>& in_sweeping_area,
                                              std::vector<std::pair<RjpPoint, RjpPoint> >& out_ori_path) {
    if (in_sweeping_area.size() < 4) {
        std::cout << "sweeping area is not a polygon in getTurnPoint func" << std::endl;
        return false;
    }
    
    double min_x, min_y, max_x, max_y;
    if (!getBoundOfSweepArea(in_sweeping_area, min_x, min_y, max_x, max_y)) {
        std::cout << "get bound of sweep area failed" << std::endl;
        return false;
    }

    std::pair<RjpPoint, RjpPoint> tmp_pair;
    std::vector<RjpPoint> tmp_vec; // every time enter a loop, clear this vector
    RjpPoint tmp_pt;
    bool order_flag = true;
    double cutline = min_x + _offset_distance;
    
    while (cutline < max_x - _offset_distance) {
        for (int i = 0; i < in_sweeping_area.size(); ++i) {
            if (getPointFromX(in_sweeping_area[i], in_sweeping_area[i + 1], cutline, tmp_pt)) {
                tmp_vec.push_back(tmp_pt);
            } else {
                // the cutline is parallel to edge of polygon
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
    // cv::Mat my_panel(1000, 1000, CV_8UC3, cv::Scalar(0, 0, 0));
    // for (int i = 0; i < out_ori_path.size(); ++i) {
    //     cv::Point2f tmp_cv_pt;
    //     tmp_cv_pt.x = cos(_rotate_angle) * out_ori_path[i].first.x + sin(_rotate_angle) * out_ori_path[i].first.y;
    //     tmp_cv_pt.y = sin(-_rotate_angle) * out_ori_path[i].first.x + cos(_rotate_angle) * out_ori_path[i].first.y;
    //     cv::circle(my_panel, tmp_cv_pt, 3, cv::Scalar(255, 255, 0), 1, CV_FILLED);
    //     tmp_cv_pt.x = cos(_rotate_angle) * out_ori_path[i].second.x + sin(_rotate_angle) * out_ori_path[i].second.y;
    //     tmp_cv_pt.y = sin(-_rotate_angle) * out_ori_path[i].second.x + cos(_rotate_angle) * out_ori_path[i].second.y;
    //     cv::circle(my_panel, tmp_cv_pt, 3, cv::Scalar(255, 255, 0), 1, CV_FILLED);
    // }
    // cv::imshow("turn points", my_panel);
    // cv::waitKey(0);

    return true;
}

bool BowShapedPlanner::getPointFromX(const RjpPoint& in_pt1, const RjpPoint& in_pt2, 
                                     double in_x, RjpPoint& out_pt) {
    if ((in_pt1.x - in_x) * (in_pt2.x - in_x) > 0) return false;

    // the line of in_pt1 and in_pt2 is parallel to cut line
    if (in_pt2.x == in_pt1.x) return false;

    double scale = (in_x - in_pt1.x) / (in_pt2.x - in_pt1.x);
    out_pt.x = in_x;
    out_pt.y = in_pt1.y + scale * (in_pt2.y - in_pt1.y);
    return true;
}

// bow-shape long line is parallel with y axis
// bow-shape short line is parallel with x axis
bool BowShapedPlanner::getStraightPointOfBowShape(const std::vector<std::pair<RjpPoint, RjpPoint> >& in_ori_path,
                                                  double in_long_dist, double in_short_dist,
                                                  std::vector<RjpPoint>& out_result_path) {
    std::vector<RjpPoint> tmp_path;
    for (int i = 0; i < in_ori_path.size() - 1; ++i) {
        if (!getLongLinePoint(in_ori_path[i], in_long_dist, tmp_path)) {    
            std::cout << ": get long line point failed" << std::endl;
            return false;
        }
        out_result_path.insert(out_result_path.end(), tmp_path.begin(), tmp_path.end());
        if (!getShortLinePoint(in_ori_path[i], in_ori_path[i + 1], in_short_dist, tmp_path)) {
            std::cout << ": get short line point failed" << std::endl;
            return false;
        }
        out_result_path.insert(out_result_path.end(), tmp_path.begin(), tmp_path.end());
    }

    if (!getLongLinePoint(in_ori_path[in_ori_path.size() - 1], in_long_dist, tmp_path)) {
        std::cout << "the last get long line point failed" << std::endl;
        return false;
    }
    
    out_result_path.insert(out_result_path.end(), tmp_path.begin(), tmp_path.end());
    out_result_path.push_back(in_ori_path[in_ori_path.size() - 1].second);
    return true;
}

// the straight points in long bow-shape line range from [start_pt, end_pt)
bool BowShapedPlanner::getLongLinePoint(const std::pair<RjpPoint, RjpPoint>& in_pt_pair,
                                        double long_dist,
                                        std::vector<RjpPoint>& out_part_traject) {
    out_part_traject.clear();

    if (in_pt_pair.first.x != in_pt_pair.second.x) {
        std::cout << "the given pt pair's X is not equal" << std::endl;
        return false;
    }

    double length = in_pt_pair.second.y - in_pt_pair.first.y;
    double length_flag = length;
    if (length < 0) {
        long_dist = -long_dist;
    }

    RjpPoint tmp_pt = in_pt_pair.first;
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
bool BowShapedPlanner::getShortLinePoint(const std::pair<RjpPoint, RjpPoint>& in_pt_pair_01,
                                         const std::pair<RjpPoint, RjpPoint>& in_pt_pair_02,
                                         double short_dist,
                                         std::vector<RjpPoint>& out_part_traject) {
    out_part_traject.clear();
    double length = in_pt_pair_02.first.x - in_pt_pair_01.second.x;
    double slope = (in_pt_pair_02.first.y - in_pt_pair_01.second.y) / length;
    double angle = getAngle(in_pt_pair_01.second, in_pt_pair_02.first);
    double length_flag = length;
    double short_dist_copy = short_dist;
    if (length < 0) {
        short_dist = -short_dist;
    }
    
    RjpPoint tmp_pt = in_pt_pair_01.second;
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

double BowShapedPlanner::getAngle(const RjpPoint& pt1, const RjpPoint& pt2) {
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