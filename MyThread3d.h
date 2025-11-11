#ifndef MYTHREAD3D_H
#define MYTHREAD3D_H
#include <QtCore/QCoreApplication>
#include <QThread>
#include <QDebug>

#include "Rectify.h"
#include "PhaserGrayCode.h"
#include "Matcher.h"
#include "config.h"

class MyThread3d
{

public:
	void run(); // 线程入口函数
	//void inite();

private:

	Rectify myrectify;
	PhaserGrayCode phaserGrayCode;
	Matcher matcher;
	PATH path; PARA para;
	
	//
	cv::Mat phaL, phaR, phaL_C, phaR_C, B;
	cv::Mat dif_map, depth_map;
	vector<pair<cv::Point2f, cv::Point2f>> cps;
	vector<string> filesL, filesR;

};

#endif 

