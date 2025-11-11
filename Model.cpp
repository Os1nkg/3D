
#include "Model.h"
//#include "utils.h"
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/cloud_viewer.h>
using namespace pcl;
using namespace cv;
using namespace std;



bool Model::calcDistance2(cv::Mat& dif_map, cv::Mat& depth_map, float min_z, float max_z) 
{

   
    // depth_map: 映射后存储三维坐标的图像
    cv::reprojectImageTo3D(dif_map, depth_map, Q, false, CV_32F);
    // 处理异常值
    for (int row = 0; row < depth_map.rows; ++row) {
        for (int col = 0; col < depth_map.cols; ++col) {
            float z = depth_map.at<cv::Vec3f>(row, col)[2];
            if (z < min_z || z > max_z || dif_map.at<float>(row, col) <= 0) {
                depth_map.at<cv::Vec3f>(row, col) = cv::Vec3f(0, 0, 0);
            }
        }
    }

    return true;
}


bool Model::calcDistance(cv::Mat &dif_map, cv::Mat &depth_map, vector<pair<cv::Point2f, cv::Point2f>> &cps, float min_z, float max_z) {
    
    for (auto &cp: cps)
    {
        cv::Point2f l = cp.first, r = cp.second;
        float d = l.x - r.x;
        if (d < 0){
            d = 0.0;
        }
        dif_map.at<float>(int(l.y), int(l.x)) = d;
    }

    // depth_map: 映射后存储三维坐标的图像
    cv::reprojectImageTo3D(dif_map, depth_map, Q, false, CV_32F);
    // 处理异常值
    for (int row = 0; row < depth_map.rows; ++row) {
        for (int col = 0; col < depth_map.cols; ++col) {
            float z = depth_map.at<cv::Vec3f>(row, col)[2];
            if (z < min_z || z > max_z || dif_map.at<float>(row, col) <= 0){
                depth_map.at<cv::Vec3f>(row, col) = cv::Vec3f(0, 0, 0);
            }
        }
    }

    return true;
}




bool Model::init(cv::Mat &Q) {
    this->Q = Q;
    return true;
}





bool Model::saveXYZ(const string &filename, cv::Mat &depth_map) 
{
    const double max_z = 1.0e4;
    FILE* fp = fopen(filename.data(), "wt");
    for (int y = 0; y < depth_map.rows; y++)
    {
        for (int x = 0; x < depth_map.cols; x++)
        {
            float point_x = depth_map.at<cv::Vec3f>(y, x)[0];
            float point_y = depth_map.at<cv::Vec3f>(y, x)[1];
            float point_z = depth_map.at<cv::Vec3f>(y, x)[2];
            if (fabs(point_z) > max_z || fabs(point_x) > max_z || fabs(point_y) > max_z)
            {
                point_x = 0;
                point_y = 0;
                point_z = 0;
                depth_map.at<cv::Vec3f>(y, x)[0] = 0;
                depth_map.at<cv::Vec3f>(y, x)[0] = 0;
                depth_map.at<cv::Vec3f>(y, x)[0] = 0;
            }
            fprintf(fp, "%f %f %f\n", point_x, point_y, point_z);
        }
    }
    fclose(fp);
    cout << "写入点云数据到文件:" << filename << endl;
    return true;
}



bool Model::savePLY(const string& filename, cv::Mat& depth_map)
{
    const double max_z = 1.0e4;
    //FILE* fp = fopen(filename.data(), "wt");

    
    PointCloud<PointXYZ> cloud_a;
    PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);

    cloud_a.height = depth_map.rows;
    cloud_a.width = depth_map.cols;

    cloud_a.points.resize(cloud_a.width * cloud_a.height);
    unsigned int num = 0; // u* colNumber + v;


    for (int y = 0; y < depth_map.rows; y++)
    {
        for (int x = 0; x < depth_map.cols; x++)
        {
            num = y* depth_map.rows + x;

            float point_x = depth_map.at<cv::Vec3f>(y, x)[0];
            float point_y = depth_map.at<cv::Vec3f>(y, x)[1];
            float point_z = depth_map.at<cv::Vec3f>(y, x)[2];
            if (fabs(point_z) > max_z || fabs(point_x) > max_z || fabs(point_y) > max_z)
            {
                point_x = 0;
                point_y = 0;
                point_z = 0;
               
            }
            
            cloud_a.points[num].x = point_x;
            cloud_a.points[num].y = point_y;
            cloud_a.points[num].z = point_z;

        }
    }

    *cloud = cloud_a;
    //
    char* path = new char[filename.size() + 1];
    strcpy(path, filename.c_str());
    std::cerr << "Path is : " << path << " ." << std::endl;

    //写出点云图
    PLYWriter writer;
    writer.write(path, *cloud, true);
    std::cerr << "PointCloud has : " << cloud->width * cloud->height << " data points." << std::endl;
   
   

    return true;
}


