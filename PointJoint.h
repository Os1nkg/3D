#ifndef POINTJOINT_H
#define POINTJOINT_H

#include <Eigen/Core>

#include <Eigen/Geometry>

#include <Eigen/SVD>
#include <Eigen/Dense>

#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

class PointJoint 
{
   
public:
    float dis_dif;

	

public:
    bool init(float pha_dif);


	//bool phaseMatchPoint(cv::Mat &phaL, cv::Mat &phaR, vector<pair<cv::Point2f, cv::Point2f>> &cps);
   
	float Ediatance(cv::Point3f& A, cv::Point3f& B);
	void Initialdata(float** Point, vector<cv::Point3f>& Left);



	void pose_estimation_3d3d(const vector<Point3f>& pts1, const vector<Point3f>& pts2, Mat& R, Mat& t);



	void PointMatch3(vector<cv::Point3f>& Left, vector<cv::Point3f>& Right, vector<cv::Point3f>& Leftout, vector<cv::Point3f>& Rightout);
	void MinDistance3(vector<cv::Point3f>& Left, vector<vector<cv::Point3f>>& LeftA);

	void PointMatch2(vector<cv::Point3f>& Left, vector<cv::Point3f>& Right, vector<cv::Point3f>& Leftout, vector<cv::Point3f>& Rightout);
	void MinDistance2(vector<cv::Point3f>& Left, vector<vector<cv::Point3f>>& LeftA);

	void PointMatch(vector<cv::Point3f> &Left, vector<cv::Point3f> &Right, vector<cv::Point3f>& Leftout, vector<cv::Point3f>& Rightout);
	void Point3DMatch(vector<cv::Point3f>& Left, vector<cv::Point3f>& Right, vector<cv::Point3f>& Leftout, vector<cv::Point3f>& Rightout);
	
	void PointFuse(vector<Point3f>& Left, vector<Point3f>& Right, Mat& R, Mat& t);
	void FindXYP(vector<cv::Point3f>& Input1, vector<cv::Point3f>& Input2, vector<int>& Lable);

//protected:
    


};


#endif 
