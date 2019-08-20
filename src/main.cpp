#include <iostream>

#include "bow_shaped_planner.h"
#include "decomposition.h"

void createSweepingArea(std::vector<cv::Point2f>& sweeping_area_cv, std::vector<ros_msgs::Vector2>& sweep_area);
void drawSweepAreanAndPath(const std::vector<cv::Point2f>& sweeping_area_cv, const ros_msgs::Trajectory& traject_ros);

int main(int argc, char* argv[]) {    
    // for visualization
    std::vector<cv::Point2f> sweeping_area_cv;
    std::vector<cv::Point2f> traject_cv;
    // for calculation
    std::vector<ros_msgs::Vector2> sweeping_area;
    ros_msgs::Trajectory traject;
    ros_msgs::Odometry odom;

    // initial the sweeping_area 
    createSweepingArea(sweeping_area_cv, sweeping_area);

    std::vector<std::vector<ros_msgs::Vector2> > split_polygons;
    PolygonDecomposition* pde = new PolygonDecomposition();
    pde->decomposePolygon(sweeping_area, split_polygons);
    std::cout << "success" << std::endl;

/*   
    BowShapedPlanner* bsp = new BowShapedPlanner();
    if (!bsp->coveragePlan(odom, sweeping_area, traject)) {
        std::cout << "converge plan failed" << std::endl;
        return 0;
    }
    std::cout << traject.poses.size() << std::endl;
    std::cout << "start to draw the final result" << std::endl;
    drawSweepAreanAndPath(sweeping_area_cv, traject);

    if (bsp) {
        delete bsp;
        bsp = NULL;
    }
*/
    return 0;
}

void createSweepingArea(std::vector<cv::Point2f>& sweeping_area_cv, std::vector<ros_msgs::Vector2>& sweep_area) {
/*
    // test 01
    sweeping_area_cv.push_back(cv::Point2f(100, 300));
    sweeping_area_cv.push_back(cv::Point2f(150, 200));
    sweeping_area_cv.push_back(cv::Point2f(100, 100));
    sweeping_area_cv.push_back(cv::Point2f(400, 100));
    sweeping_area_cv.push_back(cv::Point2f(350, 200));
    sweeping_area_cv.push_back(cv::Point2f(400, 300));
*/  

/*
    // test 02
    sweeping_area_cv.push_back(cv::Point2f(100, 100));
    sweeping_area_cv.push_back(cv::Point2f(100, 300));
    sweeping_area_cv.push_back(cv::Point2f(200, 300));
    sweeping_area_cv.push_back(cv::Point2f(200, 500));
    sweeping_area_cv.push_back(cv::Point2f(500, 500));
    sweeping_area_cv.push_back(cv::Point2f(500, 100));
*/

    sweeping_area_cv.push_back(cv::Point2f(100, 400));
    sweeping_area_cv.push_back(cv::Point2f(200, 300));
    sweeping_area_cv.push_back(cv::Point2f(100, 100));
    sweeping_area_cv.push_back(cv::Point2f(400, 100));
    sweeping_area_cv.push_back(cv::Point2f(400, 200));
    sweeping_area_cv.push_back(cv::Point2f(500, 200));
    sweeping_area_cv.push_back(cv::Point2f(300, 300));
    sweeping_area_cv.push_back(cv::Point2f(500, 400));
    
    for (int i = 0; i < sweeping_area_cv.size(); ++i) {
        ros_msgs::Vector2 tmp_vec;
        tmp_vec.x = sweeping_area_cv[i].x;
        tmp_vec.y = sweeping_area_cv[i].y;
        sweep_area.push_back(tmp_vec);
    }
    cv::Mat my_panel(1000, 1000, CV_8UC3, cv::Scalar(40, 0, 0));
    // cv::polylines(my_panel, sweeping_area_cv, true, cv::Scalar(45, 90, 135), 4);
    for (int i = 0; i < sweeping_area_cv.size() - 1; ++i) {
        cv::circle(my_panel, sweeping_area_cv[i], 3, cv::Scalar(200, 0 ,0), CV_FILLED);
        cv::line(my_panel, sweeping_area_cv[i], sweeping_area_cv[i + 1], cv::Scalar(0, 200, 0), 1);
    }
    cv::circle(my_panel, sweeping_area_cv[sweeping_area_cv.size() - 1], 3, cv::Scalar(200, 0 ,0), CV_FILLED);
    cv::line(my_panel, sweeping_area_cv[sweeping_area_cv.size() - 1], sweeping_area_cv[0], cv::Scalar(0, 200, 0), 1);
    cv::imshow("sweeping area", my_panel);
    cv::waitKey(0);
}

void drawSweepAreanAndPath(const std::vector<cv::Point2f>& sweeping_area_cv, 
                           const ros_msgs::Trajectory& traject_ros) {
    // transform the tracject_ros to traject_cv
    std::vector<cv::Point2f> traject_cv;
    cv::Point2f tmp_pt;
    for (int i = 0; i < traject_ros.poses.size(); ++i) {
        tmp_pt.x = traject_ros.poses[i].position.x;
        tmp_pt.y = traject_ros.poses[i].position.y;
        traject_cv.push_back(tmp_pt);
    }
    std::cout << "traject_cv size " << traject_cv.size() << std::endl;
    // std::vector<std::vector<cv::Point2f> > ffff;
    // ffff.push_back(traject_cv);
    // std::cout << traject_cv[traject_cv.size() - 1].x << " " << traject_cv[traject_cv.size() - 1].y << std::endl;
    cv::Mat my_panel(1000, 1000, CV_8UC3, cv::Scalar(40, 0, 0));

    // draw sweeping area
    for (int i = 0; i < sweeping_area_cv.size() - 1; ++i) {
        cv::circle(my_panel, sweeping_area_cv[i], 3, cv::Scalar(200, 0 ,0), CV_FILLED);
        cv::line(my_panel, sweeping_area_cv[i], sweeping_area_cv[i + 1], cv::Scalar(0, 200, 0), 2);
    }
    cv::circle(my_panel, sweeping_area_cv[sweeping_area_cv.size() - 1], 3, cv::Scalar(200, 0 ,0), CV_FILLED);
    cv::line(my_panel, sweeping_area_cv[sweeping_area_cv.size() - 1], sweeping_area_cv[0], cv::Scalar(0, 200, 0), 2);
    // cv::polylines(my_panel, sweeping_area_cv, false, cv::Scalar(45, 90, 135), 4, 0, 0);
    
    // draw trajectory
    for (int i = 0; i < traject_cv.size() - 1; ++i) {
        cv::circle(my_panel, traject_cv[i], 3, cv::Scalar(0, 0, 255), CV_FILLED);
        cv::line(my_panel, traject_cv[i], traject_cv[i + 1], cv::Scalar(255, 255, 0), 1);
    }
    cv::circle(my_panel, traject_cv[traject_cv.size() - 1], 3, cv::Scalar(0, 0, 255), CV_FILLED);
    // cv::line(my_panel, traject_cv[traject_cv.size() - 1], traject_cv[0], cv::Scalar(255, 255, 0), 1);
    // cv::polylines(my_panel, traject_cv, false, cv::Scalar(135, 180, 215), 2, 0, 0);

    cv::imshow("sweeping area and trajectory", my_panel);
    cv::waitKey(0);
}