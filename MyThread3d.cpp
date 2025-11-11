#include "MyThread3d.h"
//#include "utils.h"
#include "Model.h"




void MyThread3d::run() // 线程入口函数
{
	qDebug() <<  "run() begin";
	qDebug() << "run() begin";

	myrectify.calib(path.intrinsic_file, path.extrinsic_file, para.W, para.H);

	matcher.init(path.winSize_match, path.pha_dif_value);
	qDebug() << "run() end";
	
	// 02 解相位
	//glob(path.model_dir, filesL, filesR);
	
	phaserGrayCode.init(path.N, path.n, path.n_wb, path.I_thr);


	phaserGrayCode.calcPhase(filesL, phaL, B);
	phaserGrayCode.calcPhase(filesR, phaR, B);

	// 03 异常滤波
//    phaserGrayCode.filterB(phaL, phaL, B, config.B_min);
//    phaserGrayCode.filterB(phaR, phaR, B, config.B_min);


	phaserGrayCode.filterGradient(phaL, phaL);
	phaserGrayCode.filterGradient(phaR, phaR);


	// 04 立体校正
	myrectify.rectify(phaL, phaR, phaL_C, phaR_C);

	// 05 相位匹配
	matcher.phaseMatch(phaL_C, phaR_C, cps);

	// 06 计算距离
	dif_map = cv::Mat::zeros(phaL_C.size(), CV_32FC1);
	depth_map = cv::Mat::zeros(phaL_C.size(), CV_32FC1);
	
	Model model;
	model.init(myrectify.Q);
	model.calcDistance(dif_map, depth_map, cps, path.min_z, path.max_z);

	// 07 保存点云
	model.saveXYZ(path.save_file_point3d, depth_map);
	qDebug() << "run() end";
}
