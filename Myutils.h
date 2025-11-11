#ifndef UTILS_H
#define UTILS_H

#include<fstream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;


// 将一幅图片切分为两幅
//此处需要修改成
/*
bool d2(Mat &img, Mat &imgL, Mat &imgR)
{
    Size img_size = img.size();
    int w = int(img_size.width / 2), h = img_size.height;
    Rect RectL(0, 0, w, h); Rect RectR(w, 0, w, h);
    imgL = img(RectL); imgR = img(RectR);
    return ((!imgL.empty()) && (!imgR.empty()));
}
*/



void PMimread2(const string &filename, Mat &img, int color_model=1)
{
    img = imread(filename, color_model);
    if (img.empty()){
        cerr << "图像为空:" << filename << std::endl; assert(0);
    }
}




void PMshow(Mat &img, const string &title)
{
    imshow(title, img);
    waitKey(0);
    destroyWindow(title);
}



bool Myis_exist(const string &filename){
    ifstream f(filename.c_str());
    return f.good();
}



bool Mysave_disp(Mat &disp8, const string &filename) 
{
    ofstream fw(filename, ios::out);
    if (!fw.is_open()){
        cerr << "未成功打开视差文件" << std::endl;
        return false;
    }
    fw << disp8.rows << std::endl;
    fw << disp8.cols << std::endl;
    for (int y = 0; y < disp8.rows; y++) {
        for (int x = 0; x < disp8.cols; ++x) {
            double d = disp8.at<uint8_t>(y, x);
            fw << d << std::endl;
        }
    }
    fw.close();
    cout << "成功保存视差文件:" << filename << std::endl;
    return true;
}


bool Mysave3dPoint(Mat &xyz, const string &filename)
{
    const double max_z = 1.0e4;
    ofstream fw(filename, ios::out);
    if (!fw.is_open()){
        cerr << "写入点云文件失败!" << filename << std::endl;
        return false;
    }
    Vec3f point;
    for (int y = 0; y < xyz.rows; ++y) {
        for (int x = 0; x < xyz.cols; ++x) {
            point = xyz.at<Vec3f>(y, x);
            if (fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z){
                continue;
            }
            fw << point[0] << " " << point[1] << " " << point[2] << std::endl;
        }
    }
    fw.close();
    cout << "保存点云文件成功:" << filename << std::endl;
    return true;
}

bool Myreproject3d(Mat &disp8, Mat &xyz, Mat &Q)
{
    reprojectImageTo3D(disp8, xyz, Q, false, -1);
    return true;
}


bool Mysplit(const string &line, vector<double> &res, const string &pattern)
{
	if (line.empty())
		return false;
	res.clear();
	stringstream ss(line);
	string data;
	double v;
	char *ptr;
	while (getline(ss, data, '\t')) {
		// 将读取的数据转换为
		v = strtod(data.c_str(), &ptr);
		res.emplace_back(v);
	}
	return !res.empty();
}



bool Myglob(const string &folder, vector<string> &filesL, vector<string> &filesR)
{

	filesL.clear(); filesR.clear();
	string folderL = folder + "\\L";
	string folderR = folder + "\\R";

	//最好是遍历文件夹，查看有多少张图像
	// 采用格雷码 + 相移进行编码
	for (int i = 0; i < 10; ++i)
	{
		int idx = i + 1;
		string filename_L = folderL + "\\" + to_string(idx) + ".bmp";
		string filename_R = folderR + "\\" + to_string(idx) + ".bmp";
		filesL.emplace_back(filename_L);
		filesR.emplace_back(filename_R);
		cout << "L:" << filename_L << "\t|\t" << "R:" << filename_R << std::endl;
	}
	return (!filesL.empty()) && (!filesR.empty());

}



void MyshowImagePair(const cv::Mat &imgL, const cv::Mat &imgR, const string &title, float s, bool line, bool color) {
	cv::Mat img_all;
	cv::hconcat(imgL, imgR, img_all);
	cv::resize(img_all, img_all, cv::Size(), s, s);
	double min_v, max_v;
	cv::Point min_p, max_p;
	cv::minMaxLoc(img_all, &min_v, &max_v, &min_p, &max_p);
	img_all = (img_all - min_v) * 255. / (max_v - min_v);
	img_all.convertTo(img_all, CV_8UC1);
	if (color) {
		cv::applyColorMap(img_all, img_all, cv::COLORMAP_JET);
	}
	// 绘制平行线
	int h = img_all.rows;
	int w = img_all.cols;
	if (line) {
		int num = 10;
		int p = h / num;
		for (int i = 0; i < num; ++i) {
			int y = i * p;
			cv::line(img_all, cv::Point(0, y), cv::Point(w, y), cv::Scalar(255, 255, 255), 1);
		}
	}
	cv::imshow(title, img_all);
	cv::waitKey(1);
}

void MyshowImage(const cv::Mat &img, const string &title, float s, bool color) {
	cv::Mat img_s;
	cv::resize(img, img_s, cv::Size(), s, s);
	double min_v, max_v;
	cv::Point min_p, max_p;
	cv::minMaxLoc(img_s, &min_v, &max_v, &min_p, &max_p);
	img_s = (img_s - min_v) * 255. / (max_v - min_v);
	img_s.convertTo(img_s, CV_8UC1);
	if (color) {
		cv::applyColorMap(img_s, img_s, cv::COLORMAP_JET);
	}
	cv::imshow(title, img_s);
	cv::waitKey(1);
}


void Myimwrite(const string &filename, cv::Mat &mat)
{
	cout << "保存矩阵到:" << filename << std::endl;
	cv::imwrite(filename, mat);
}


#endif //UTILS_H
