#include "decomposition.h"
/*
    For topographic consideration, use index of each point instead of points directly
    It enable us to use Int to represent a point along with the point's topography
    The key to understand these codes is to distinguish what the index points to
    Which is the core of GIS thinking

    Specifically, we have origin_polygon, index_polygon
    origin_polygon is the data set, index_polygon is the index of the origin_polygon
    You have to make sure the index is for origin_polygon or index_polygon

*/

PolygonDecomposition::PolygonDecomposition () {

}

PolygonDecomposition::~PolygonDecomposition () {
    
}

// interface
// input, output data type is `RjpPoint`
// use cv::Point in computing for the convenience of OpenCV lib 
bool PolygonDecomposition::decomposePolygon(const std::vector<RjpPoint>& in_polygon,
                                            std::vector<std::vector<RjpPoint> >& out_result_polygons) {

    if (!convertRos2CvPolygon(in_polygon)) {
        std::cout << "convertRos2CvPolygon() failed" << std::endl;
        return false;
    } 
    
    if (!decomposeIt()) {
        std::cout << "decomposeIt() failed" << std::endl;
        return false;
    }

    if (!convertCv2RosPolygon(out_result_polygons)) {
        std::cout << "convertCv2RosPolygon() failed" << std::endl;
        return false;
    }

    std::cout << "decomposePolygon() succeeded" << std::endl;
    return true;
}

// find the xmin or xmax or ymin or ymax point in polygon
// that's feature point P_{i} which determines the polygon's direction (clockwise or counterclockwise)
// use vec(P_{i-1}, P{i}) cross vec(P_{i}, P_{i+1}) to judge polygon's direction 
// save `in_polygon` in the private varible `_polygon_cv` counterclockwise
// initial private varible `_result_polygon_idx` which carries the index of all polygons
bool PolygonDecomposition::convertRos2CvPolygon(const std::vector<RjpPoint>& in_polygon) {
    if (in_polygon.size() < 3) {
        std::cout << "the input is not a polygon with the size of " << in_polygon.size() << std::endl;
        return false;
    }

    // find the ymin
    auto ymin_iter = std::min_element(in_polygon.begin(), in_polygon.end(),
                                     [](RjpPoint p1, RjpPoint p2){return p1.y < p2.y;});
    int y_min_idx = std::distance(in_polygon.begin(), ymin_iter);
    
    // calculate the cross product
    double cross_product;
    if (y_min_idx == 0) {
        cross_product = calCrossProduct(in_polygon[in_polygon.size() - 1], 
                                        in_polygon[y_min_idx], 
                                        in_polygon[y_min_idx + 1]);
    } else if (y_min_idx = in_polygon.size() - 1) {
        cross_product = calCrossProduct(in_polygon[y_min_idx - 1],
                                        in_polygon[y_min_idx],
                                        in_polygon[0]);
    } else {
        cross_product = calCrossProduct(in_polygon[y_min_idx - 1],
                                        in_polygon[y_min_idx],
                                        in_polygon[y_min_idx + 1]);
    }
    
    // arrange the polygon counterclockwise
    if (cross_product > 0) {    // counterclockwise
        std::cout << " the polygon's direction is counterclockwise" << std::endl;
        for (int i = 0; i < in_polygon.size(); ++i) {
            _polygon_cv.push_back(cv::Point2f(in_polygon[i].x, in_polygon[i].y));
        }
    } else {    // clockwise
        std::cout << " the polygon's direction is clockwise" << std::endl;
        _polygon_cv.push_back(cv::Point2f(in_polygon[0].x, in_polygon[0].y));
        for (int i = in_polygon.size() - 1; i >0; --i) {
            _polygon_cv.push_back(cv::Point2f(in_polygon[i].x, in_polygon[i].y));
        }
    }

    // initial _result_polygon_idx
    std::vector<int> tmp_poly;
    for (int i = 0; i < _polygon_cv.size(); ++i) {
        tmp_poly.push_back(i);
    }
    _result_polygon_idx.push_back(tmp_poly);

    std::cout << " convertRos2CvPolygon() succeeded" << std::endl;
    return true;
}

// the `in_polygon_idx` index to origin_polygon
void PolygonDecomposition::getCvPolygonFromIdx(const std::vector<int>& in_polygon_idx, 
                                               std::vector<cv::Point2f>& out_polygon_cv) {
    out_polygon_cv.clear();
    // std::cout << "inpoly_idx size  " << in_polygon_idx.size() << std::endl;
    for (int i = 0; i < in_polygon_idx.size(); i++) {
        // std::cout << "x " << _polygon_cv[in_polygon_idx[i]].x;
        // std::cout << " y " << _polygon_cv[in_polygon_idx[i]].y << std::endl;
        out_polygon_cv.push_back(_polygon_cv[in_polygon_idx[i]]);
    }
}

// use OpenCV to judge whether the polygon is a convex as a double check
// current polygon is arranged counterclockwise
// so if the cross product of vec(p-1, p) and vec(p, p+1) is less than 0
// the point `p` is a concave point
bool PolygonDecomposition::getConcavePointsIdx(const std::vector<int>& in_idx, 
                                               std::vector<int>& out_pts_idx) {
    // double check the `in_idx` polygon 
    out_pts_idx.clear();

    std::vector<cv::Point2f> in_concave_polygon_cv;
    getCvPolygonFromIdx(in_idx, in_concave_polygon_cv);

    if (cv::isContourConvex(in_concave_polygon_cv)) {
        std::cout << "the polygon is convex" << std::endl;
        // std::cout << "getConcavePoints() failed " << std::endl;
        return false;
    }

    for (int i = 0; i < in_idx.size(); ++i) {
        double cross_product;
        if (i == 0) {   // first point
            cross_product = calCrossProduct(_polygon_cv[in_idx[in_idx.size() - 1]],
                                            _polygon_cv[in_idx[i]],
                                            _polygon_cv[in_idx[i + 1]]);
        } else if (i == in_idx.size() - 1) {   // last point
            cross_product = calCrossProduct(_polygon_cv[in_idx[i - 1]],
                                            _polygon_cv[in_idx[i]],
                                            _polygon_cv[in_idx[0]]);
        } else {    // other points
            cross_product = calCrossProduct(_polygon_cv[in_idx[i - 1]],
                                            _polygon_cv[in_idx[i]],
                                            _polygon_cv[in_idx[i + 1]]);
        }

        if (cross_product < 0) {    // convex
            out_pts_idx.push_back(i);
        }
    }

    // double check
    if (out_pts_idx.size() == 0) {
        std::cout << "the concave point size is 0 " << std::endl;
        return false;
    }

    std::cout << "getConcavePoints() succeeded" << std::endl;
    return true;
}

// construct the `_result_polygon_idx` which index the arranged `_polygon_cv` 
// simplify the conventiol algorithm to statisfy my demand
// update `_result_polygon_idx` when there is still a concave polygon in it
// compute `_output_polygon_idx` to save the decomposed multi-convex-polygons
bool PolygonDecomposition::decomposeIt() {
    int count = 0;
    std::vector<std::vector<int> > copy_result_polygon_idx = _result_polygon_idx;
    std::vector<int> concave_polygon_idx;
    while (!isAllConvex(concave_polygon_idx)) {
        std::cout << "there is " << concave_polygon_idx.size() << " concave now" << std::endl;
        for (int i = 0; i < concave_polygon_idx.size(); ++i) {
            std::vector<int> concave_poly = _result_polygon_idx[concave_polygon_idx[i]];
            std::vector<int> concave_pts_idx;
            if (!getConcavePointsIdx(concave_poly ,concave_pts_idx)) {
                std::cout << "getConcavePointsIdx() failed" << std::endl;
                return false;
            }
            
            int first_pt_idx = concave_pts_idx[0];   // first concave point idx counterclockwise
            std::cout << "the first concavee pt is " << _polygon_cv[concave_poly[first_pt_idx]] << std::endl;

            std::vector<int> visible_pts_idx;
            if (!findVisiblePointsInSearchRange(concave_poly, first_pt_idx, visible_pts_idx)) {
                std::cout << "findVisiblePointsInSearchRange() failed" << count << std::endl;
            }
            if (visible_pts_idx.size() == 0) {  // actually, there is at least one point
                std::cout << "visible_pts_idx size is 0" << std::endl;
                return false;
            }

            std::cout << "the visible pts are" <<std::endl; 
            for (int i = 0; i < visible_pts_idx.size(); ++i) {
                std::cout << _polygon_cv[concave_poly[visible_pts_idx[i]]];
            }
            std::cout << std::endl;

            std::vector<double> power_vector;
            calPowerOfVisiblePoints(concave_poly, first_pt_idx, visible_pts_idx, power_vector);

            std::cout << "the power vector is " << std::endl;
            for (int i = 0; i < power_vector.size(); ++i) {
                std::cout << i << " " << power_vector[i] << std::endl;
            }

            auto max_value = std::max_element(power_vector.begin(), power_vector.end());
            auto max_idx = std::distance(power_vector.begin(), max_value);
            int split_pt_idx = visible_pts_idx[max_idx];

            std::vector<std::vector<int> > split_polygons;
            getSplitPolygons(concave_poly, first_pt_idx, split_pt_idx, split_polygons);
            if (split_polygons.size() != 2) {
                std::cout << "the split polygons size is not 2 " << ", is " << split_polygons.size() << std::endl;
                return false;
            }

            copy_result_polygon_idx.push_back(split_polygons[0]);
            copy_result_polygon_idx.push_back(split_polygons[1]);
        }
        for (int i = 0; i < concave_polygon_idx.size(); ++i) {
            copy_result_polygon_idx.erase(copy_result_polygon_idx.begin() + concave_polygon_idx[i]);
        }
        _result_polygon_idx = copy_result_polygon_idx;
        
        std::cout << "the result is" << std::endl;
        for (int i = 0; i < _result_polygon_idx.size(); ++i) {
            for (int j = 0; j < _result_polygon_idx[i].size(); ++j) {
                int idx = _result_polygon_idx[i][j];
                std::cout << _polygon_cv[idx];
            }
            std::cout << std::endl;
        }
        std::cout << "***********************************" << std::endl;
        count++;
    }
    return true;
}

// when finished the decomposition, `_result_polygon_idx` to the output variable
bool PolygonDecomposition::convertCv2RosPolygon(std::vector<std::vector<RjpPoint> >& out_polygon_ros) {    
    out_polygon_ros.clear();
    for (int i = 0; i < _result_polygon_idx.size(); ++i) {
        std::vector<RjpPoint> sub_polygon;
        for (int j = 0; j < _result_polygon_idx[i].size(); ++j) {
            RjpPoint sub_polygon_pt;
            sub_polygon_pt.x = _polygon_cv[_result_polygon_idx[i][j]].x;
            sub_polygon_pt.y = _polygon_cv[_result_polygon_idx[i][j]].y;
            sub_polygon.push_back(sub_polygon_pt);
        }
        out_polygon_ros.push_back(sub_polygon); 
    }
    return true;
}

double PolygonDecomposition::calCrossProduct(cv::Point2f pt_01, cv::Point2f pt_02, cv::Point2f pt_03) {
    double vec_01_x = pt_02.x - pt_01.x;
    double vec_01_y = pt_02.y - pt_01.y;
    double vec_02_x = pt_03.x - pt_02.x;
    double vec_02_y = pt_03.y - pt_02.y;
    return (vec_01_x * vec_02_y - vec_01_y * vec_02_x);  
}

double PolygonDecomposition::calCrossProduct(RjpPoint pt_01, RjpPoint pt_02, RjpPoint pt_03) {
    double vec_01_x = pt_02.x - pt_01.x;
    double vec_01_y = pt_02.y - pt_01.y;
    double vec_02_x = pt_03.x - pt_02.x;
    double vec_02_y = pt_03.y - pt_02.y;
    return (vec_01_x * vec_02_y - vec_01_y * vec_02_x);
}

// the given `in_pt_idx` is the first concave point counterclockwise from the start point of the polygon
// `in_poly` index to origin_polygon `_polygon_cv`
// `in_pt_idx` index to index_polygon `_in_poly`
// so by the twice index, we can get the point's data according to an Int using `_polygon[in_poly[in_pt_idx]]`
// Visible Points definition in the paper mentioned in .h file
// `out_pts_idx` is index to index_polygon `in_poly`
bool PolygonDecomposition::findVisiblePointsInSearchRange(const std::vector<int>& in_poly, 
                                                          int in_pt_idx, 
                                                          std::vector<int>& out_pts_idx) {
    out_pts_idx.clear();

    std::vector<int> search_all_pts_idx;    // index for index_polygon `in_poly`

    if (!findSearchRange(in_poly, in_pt_idx, search_all_pts_idx)) {
        std::cout << "findSearchRange() failed" << std::endl;
        return false;
    }

    std::cout << "the search range pts" << std::endl;
    for (int i = 0; i < search_all_pts_idx.size(); ++i) {
        std::cout << _polygon_cv[in_poly[search_all_pts_idx[i]]];
    }
    std::cout << std::endl;

    if (search_all_pts_idx.size() == 0) {
        std::cout << "findVisiblePointsInSearchRange(): No visible points in search range" << std::endl;
        return false;
    }

    for (int i = 0; i < search_all_pts_idx.size(); ++i) {
        if (judgeVisibility(in_poly, in_pt_idx, search_all_pts_idx[i])) {
            out_pts_idx.push_back(search_all_pts_idx[i]);
        }
    }
    return true;
}

// Point A(X_a, Y_a), Point B(X_b, Y_b)
// F(x, y) = (Y_b - Y_a)x + (X_a - X_b)y + (Y_a * X_b - X_a * Y_b)
// F(x, y) > 0, the Point(x, y) is in the right region of vector(A, B)
// otherwise in the right region
// in this function, we search for the left region
// `out_points_idx` is index for index_polygon `in_poly`
bool PolygonDecomposition::findSearchRange(const std::vector<int>& in_poly,
                                           int in_pt_idx, 
                                           std::vector<int>& out_points_idx) {
    out_points_idx.clear();

    int A_idx = in_pt_idx;
    int B_idx = (A_idx == in_poly.size() - 1) ? 0 : A_idx + 1;  // the A's next point counterclockwise

    double A_x, B_y, C_c;
    A_x = _polygon_cv[in_poly[B_idx]].y - _polygon_cv[in_poly[A_idx]].y;
    B_y = _polygon_cv[in_poly[A_idx]].x - _polygon_cv[in_poly[B_idx]].x;
    C_c = _polygon_cv[in_poly[A_idx]].y * _polygon_cv[in_poly[B_idx]].x - _polygon_cv[in_poly[A_idx]].x * _polygon_cv[in_poly[B_idx]].y;

    for (int i = 0; i < in_poly.size(); ++i) {
        if (i != A_idx && i != B_idx) {
            if  ((A_x * _polygon_cv[in_poly[i]].x + B_y * _polygon_cv[in_poly[i]].y + C_c) < 0) { // less than 0 is left region
                out_points_idx.push_back(i);
            }
        }
    }
    return true;
}       

// TODO: a little problem here, though it works for the complicated situation 
// the way I implement this is really stupid and low efficient
// When I search the methods to judge the visible points of polygon
// the algorithm is fucking huge!!!
bool PolygonDecomposition::judgeVisibility(const std::vector<int>& in_poly, 
                                           int in_concave_pt_idx, int in_search_pt_idx) {
    int pt_A = in_concave_pt_idx;
    int pt_A_front = (pt_A == 0) ? (in_poly.size() - 1) : (pt_A - 1);
    int pt_B = in_search_pt_idx;
    int pt_B_front = (pt_B == 0) ? (in_poly.size() - 1) : (pt_B - 1);

    cv::Point2f line01_start = _polygon_cv[in_poly[pt_A]];
    cv::Point2f line01_end = _polygon_cv[in_poly[pt_B]];

    for (int i = 0; i < in_poly.size(); ++i) {
        if (i == pt_A || i == pt_A_front || i == pt_B || i == pt_B_front) {
            continue;
        }

        int i_next = (i == in_poly.size() - 1) ? 0 : i + 1 ;
        cv::Point2f line02_start = _polygon_cv[in_poly[i]];
        cv::Point2f line02_end = _polygon_cv[in_poly[i_next]];
        cv::Point2f intersect_pt;
        if (getIntersectPoints(line01_start, line01_end, line02_start, line02_end, intersect_pt)) {
            if ((line01_end.x - intersect_pt.x) * (line01_start.x - intersect_pt.x) <= 0 &&
                (line02_end.x - intersect_pt.x) * (line02_start.x - intersect_pt.x) < 0) {
                    // the 2 line segments intersect
                    return false;
                }
        } else {
            // the lines parallel
        }
    }
    return true;
}

// clear the output `out_concave_polygon_idx`
// judge polygon in `_result_polygon_idx`
bool PolygonDecomposition::isAllConvex(std::vector<int>& out_concave_polygon_idx) {
    out_concave_polygon_idx.clear();

    bool is_all_convex = true;
    // std::cout << "_result_polygon_idx.size() " << _result_polygon_idx.size() << std::endl;
    for (int i = 0; i < _result_polygon_idx.size(); ++i) {
        // take out the polygon
        std::vector<cv::Point2f> tmp_polygon_cv;
        getCvPolygonFromIdx(_result_polygon_idx[i], tmp_polygon_cv);

        // if the polygon is concave
        if (!cv::isContourConvex(tmp_polygon_cv)) {
            out_concave_polygon_idx.push_back(i);
            is_all_convex = false;
        }
    }
    // debug
    if (out_concave_polygon_idx.size() == 0) {
        std::cout << " there is 0 concave polygons " << std::endl;
        return true;
    }

    return is_all_convex;
}

// this algorithm is proposed by 
// https://stackoverrun.com/cn/q/1900650
// an efficient way to compute the intersection of 2 lines
bool PolygonDecomposition::getIntersectPoints(cv::Point2f line01_start, cv::Point2f line01_end,
                                              cv::Point2f line02_start, cv::Point2f line02_end,
                                              cv::Point2f& intersect_pt) {
    cv::Point2f x = line02_start - line01_start;
    cv::Point2f d1 = line01_end - line01_start;
    cv::Point2f d2 = line02_end - line02_start;

    float cross = d1.x * d2.y - d1.y * d2.x;
    if (fabs(cross) < 1e-8) return false;
    double t1 = (x.x * d2.y - x.y * d2.x) / cross;
    intersect_pt = line01_start + d1 * t1;
    return true;                                              
}

// the full mark of power is 200
// if the visible point is a concave, power adds 100, the other 100 is 100 times the absolute of 2 lines' cosine
// that guarantees if there is concave point, the power of it will be the highest
bool PolygonDecomposition::calPowerOfVisiblePoints(const std::vector<int>& in_poly, 
                                                   int in_concave_idx,
                                                   const std::vector<int>& in_pts_idx, 
                                                   std::vector<double>& out_power_vector) {
    out_power_vector.clear();

    // get concave point bisector
    int center_idx = in_concave_idx;
    int left_idx = (center_idx == 0) ? in_poly.size() -1 : center_idx - 1 ;
    int right_idx = (center_idx == in_poly.size() - 1) ? 0 : center_idx + 1;
    double left_x = _polygon_cv[in_poly[left_idx]].x - _polygon_cv[in_poly[center_idx]].x;
    double left_y = _polygon_cv[in_poly[left_idx]].y - _polygon_cv[in_poly[center_idx]].y;
    double right_x = _polygon_cv[in_poly[right_idx]].x - _polygon_cv[in_poly[center_idx]].x;
    double right_y = _polygon_cv[in_poly[right_idx]].y - _polygon_cv[in_poly[center_idx]].y;
    Eigen::Vector2d left_vec;
    left_vec << left_x, left_y;
    Eigen::Vector2d right_vec;
    right_vec << right_x, right_y;
    left_vec.normalize();
    right_vec.normalize();
    Eigen::Vector2d bisector_vec = left_vec + right_vec;
    bisector_vec.normalize();

    for (int i = 0; i < in_pts_idx.size(); ++i) {
        double p_value = 0;
        // judge `in_poly[in_pts_idx[i]]` is a  concave
        int c_idx = in_pts_idx[i];
        int l_idx = (c_idx == 0) ? in_poly.size() - 1 : c_idx - 1;
        int r_idx = (c_idx == in_poly.size() - 1) ? 0 : c_idx + 1;
        cv::Point2f l2c_vec = _polygon_cv[in_poly[c_idx]] - _polygon_cv[in_poly[l_idx]];
        cv::Point2f c2r_vec = _polygon_cv[in_poly[r_idx]] - _polygon_cv[in_poly[c_idx]];
        double cross = l2c_vec.x * c2r_vec.y - l2c_vec.y * c2r_vec.x;
        if (cross < 0) {    // this point is concave, add the power with 100;
            p_value += 100;
        }
        double del_x = _polygon_cv[in_poly[c_idx]].x - _polygon_cv[in_poly[center_idx]].x;
        double del_y = _polygon_cv[in_poly[c_idx]].y - _polygon_cv[in_poly[center_idx]].y;
        Eigen::Vector2d line_vec;
        line_vec << del_x, del_y;
        line_vec.normalize();
        p_value += fabs(bisector_vec.dot(line_vec)) * 100;
        out_power_vector.push_back(p_value);
    } 
    return true;                
}

// ATTENTION: `out_polys` is index to origin_polygon `_polygon_cv` stored in `_result_polygons_idx`
// Since the polygon is arranged in counterclockwise
// when you get the 2 split points one of which is the first concave point 
// it's easy to split the concave polygons into 2 polygons
bool PolygonDecomposition::getSplitPolygons(const std::vector<int>& in_poly, int in_concave_pt_idx, int in_split_pt_idx,
                                            std::vector<std::vector<int> >& out_polys) {
    out_polys.clear();
    
    if (in_concave_pt_idx == in_split_pt_idx) {
        std::cout << "concave idx equals split idx" << std::endl;
        std::cout << "getSplitPolygons() failed" << std::endl;
        return false;
    }

    std::vector<int> region_01, region_02;
    int start_idx = (in_concave_pt_idx < in_split_pt_idx) ? in_concave_pt_idx : in_split_pt_idx;
    int end_idx = (in_concave_pt_idx > in_split_pt_idx) ? in_concave_pt_idx : in_split_pt_idx;
    // the `in_poly` is already counterclockwise
    // region_01
    for (int i = start_idx; i <= end_idx; ++i) {
        region_01.push_back(in_poly[i]);
    }
    // region_02
    for (int i = end_idx; i < in_poly.size(); ++i) {
        region_02.push_back(in_poly[i]);
    }
    for (int i = 0; i <= start_idx; ++i) {
        region_02.push_back(in_poly[i]);
    }
    // check the num, use when debug
    if ((in_poly.size() + 2) != (region_01.size() + region_02.size())) {
        std::cout << "(in_poly.size() + 2) != (region_01.size() + region_02.size())" << std::endl;
        return false;
    }

    out_polys.push_back(region_01);
    out_polys.push_back(region_02);

    return true;
}