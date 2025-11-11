
#ifndef RECTIFY_H
#define RECTIFY_H

#include<iostream>
#include <opencv2/opencv.hpp>
using namespace std;


//#include "utils.h"

//
class Rectify
{
public:
    cv::Mat Q;
protected:
    string calib_file;  // 标定文件

	/*
    cv::Mat
    KK_L, KK_R,                                     // 相机畸变
    RadialDistortion_L, RadialDistortion_R,         // 径向畸变
    TangentialDistortion_L, TangentialDistortion_R, // 切向畸变
    R, T, E, F, error, Dist_L, Dist_R;
	*/


	//
	//平移旋转矩阵等
	cv::Mat M_L, M_R, D_L, D_R;

	//图像的有效区域
	//Rect validROI_L, validROI_R;
	cv::Mat  R, T, rt_L, rt_R, P_L, P_R;
	cv::Mat mapx_L, mapy_L, mapx_R, mapy_R, disp;


    cv::Size imgSize;
    //cv::Mat rmap_L[2], rmap_R[2];
    //cv::Mat R_L, R_R, P_L, P_R;
    cv::Rect validROI_L, validROI_R;


public:
    // 加载txt文件，保存为mat矩阵，并且进行标定
    bool calib(const string &intrinsic, const string &extrinsic, int W, int H);

    // 校正双目图像
    bool rectify(cv::Mat &imgL_src, cv::Mat &imgR_src, cv::Mat &imgL_des, cv::Mat &imgR_des);

	bool calcDistance2(cv::Mat& dif_map, cv::Mat& depth_map, cv::Mat& mask, float min_z, float max_z);

	bool calcDistance(cv::Mat& dif_map, cv::Mat& depth_map, vector<pair<cv::Point2f, cv::Point2f>>& cps, float min_z, float max_z);

	bool saveXYZ(const std::string& filename, cv::Mat& depth_map);

	bool savePLY(const std::string& filename, cv::Mat& depth_map);

private:


	//加载内参文件
	
	bool load_intrinsic(const string &filename)
	{
		cv::FileStorage fr(filename, cv::FileStorage::READ);
		if (!fr.isOpened()) {
			cerr << "内参文件未打开:" << filename << std::endl;
			return false;
		}
		fr["M_L"] >> M_L; fr["D_L"] >> D_L;
		fr["M_R"] >> M_R; fr["D_R"] >> D_R;
		return true;
	}

	bool load_rectify(const string& filename)
	{
		cv::FileStorage fr(filename, cv::FileStorage::READ);
		if (!fr.isOpened()) {
			cerr << "内参文件未打开:" << filename << std::endl;
			return false;
		}
		fr["mapx_L"] >> mapx_L; fr["mapy_L"] >> mapy_L;
		fr["mapx_R"] >> mapx_R; fr["mapy_R"] >> mapy_R;
		return true;
	}

	//加载外参文件
	bool load_extrinsic(const string &filename)
	{
		cv::FileStorage fr(filename, cv::FileStorage::READ);
		if (!fr.isOpened()) {
			cerr << "外参文件未打开:" << filename << std::endl;
			return false;
		}
		else {
			fr["R"] >> R; fr["T"] >> T;
			return true;
		}
	}
	

};


#endif 
