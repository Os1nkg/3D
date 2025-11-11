#ifndef MODEL_H
#define MODEL_H
#include <opencv2/opencv.hpp>



class Model
{
    cv::Mat Q;
public:


    bool init(cv::Mat &Q);

    bool calcDistance(cv::Mat &dif_map, cv::Mat &depth_map, vector<pair<cv::Point2f, cv::Point2f>> &cps, float min_z, float max_z);

    bool calcDistance2(cv::Mat& dif_map, cv::Mat& depth_map, float min_z, float max_z);


    bool saveXYZ(const std::string &filename, cv::Mat &depth_map);

    bool savePLY(const std::string& filename, cv::Mat& depth_map);
};


#endif 
