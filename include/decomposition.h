#include <iostream>

#include <ros_msgs/Vector2.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigen>

// decompose the polygon if it's concave, output the convex polygons index
class PolygonDecomposition {
public:
    PolygonDecomposition();     // constructor
    ~PolygonDecomposition();    // deconstructor
    // set polygon
    void resetPolygon(std::vector<ros_msgs::Vector2> in_polygon);
    // interface
    bool decomposePolygon(std::vector<ros_msgs::Vector2> in_polygon,
                          std::vector<std::vector<ros_msgs::Vector2> >& out_result_polygons);
private:
    // convert `ros_msgs::Vector2` to `cv::Point2f`, reorder the polygon counterclockwise
    bool convertRos2CvPolygon(const std::vector<ros_msgs::Vector2>& in_polygon);
    // compute the concave point of given polygon
    bool getConcavePointsIdx(const std::vector<int>& in_concave_polygon_idx, std::vector<int>& out_concave_pts_idx);
    // functional: decompose the polygon
    bool decomposeIt();
    // take out the index of `_output_polygon_idx` to fill the `ros_msgs::Vecor2` polygon
    bool convertCv2RosPolygon(std::vector<std::vector<ros_msgs::Vector2> >& out_polygon_ros);
    // calculate the cross product
    double calCrossProduct(cv::Point2f pt_01, cv::Point2f pt_02, cv::Point2f pt_03);
    double calCrossProduct(ros_msgs::Vector2 pt_01, ros_msgs::Vector2 pt_02, ros_msgs::Vector2 pt_03);
    // take out the polygon_cv from the given index in `_polygon_cv`
    void getCvPolygonFromIdx(const std::vector<int>& in_polygon_idx, std::vector<cv::Point2f>& out_polygon_cv);
    // ********************************************************
    // ******************** algorithm part ********************
    // the decomposition algorithm is proposed by ZHU chuanmin, TANG jun and XU tiangui 
    // from College of Mechanical Engineering, Tongji University, Shanghai, China
    // the paper link : https://wenku.baidu.com/view/a3ccf9abf705cc1755270974.html
    // ********************************************************
    // compute the visible points with given a concave point idx
    bool findVisiblePointsInSearchRange(const std::vector<int>& in_concave_poly, 
                                        int in_concave_pt_idx, std::vector<int>& out_visible_points_idx);
    bool findSearchRange(const std::vector<int>& in_concave_poly, int in_concave_pt_idx, std::vector<int>& out_searched_points_idx);
    bool judgeVisibility(const std::vector<int>& in_concave_poly, int in_concave_pt_idx, int in_search_point_idx);
    // TODO:
    bool calPowerOfVisiblePoints(const std::vector<int>& in_concave_poly, 
                                 int in_concave_pt_idx,
                                 const std::vector<int>& in_visible_points_idx, 
                                 std::vector<double>& out_power_vector);
    // bool calAngleBisector(int in_concave_pt_idx, double out_slope);
    
    // ture if all polygons in `_result_polygon_idx` are convex
    // compute the concave polygons' index in `_result_polygon_idx`
    bool isAllConvex(std::vector<int>& out_concave_polygon_idx);
    // return false if the 2 lines are parallel
    bool getIntersectPoints(cv::Point2f line01_start, cv::Point2f line01_end,
                            cv::Point2f line02_start, cv::Point2f line02_end,
                            cv::Point2f& intersect_pt);
    bool getSplitPolygons(const std::vector<int>& in_concave_poly, int in_concave_pt_idx, int in_split_pt_idx,              
                          std::vector<std::vector<int> >& out_polys);

private:
    std::vector<cv::Point2f> _polygon_cv;   // input one polygon in cv
    std::vector<std::vector<int> > _result_polygon_idx; // generate polygons index considering the topography
};  

