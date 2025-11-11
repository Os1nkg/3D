#ifndef CONFIG_H
#define CONFIG_H

#include <iostream>
using namespace std;


// 投影仪的分辨率
const int PROJECTOR_RESLINE = 1140;
const int PROJECTOR_RESROW = 912;

// 相机的分辨率
const int CAMERA_RESLINE = 1536;
const int CAMERA_RESROW = 2048;

struct PATH
{
	string sep = "/";
	string root_dir = R"(./)";
	string data_dir = root_dir + sep + "data";

	string data_calib = data_dir + "/calib";

	string calib_dirL = data_dir + sep + "calib/R";
	string calib_dirR = data_dir + sep + "calib/L";

	string calib_dir = data_dir + sep + "calib/L";


	string calib_dirCP = data_dir + sep + "calib/L";
	//保存相机标定的内参和外部参数
	string intrinsic_file = data_dir + sep + "intrinsic_file.yml";
	string extrinsic_file = data_dir + sep + "extrinsic_file.yml";
	string rectify_file = data_dir + sep + "rectify_file.yml";

	string cameraProjector_file = data_dir + sep + "CalibrationResult.yml";

	//输出图像
	string test_dir = data_dir + sep + "test";
	string output_dir = data_dir + sep + "outputs";

	string testfilesL = output_dir+sep+"L";
	string testfilesR = output_dir+sep + "R";

	string testfilesLT = output_dir + sep + "L";
	string testfilesRT = output_dir + sep + "R";

	string model_dirL = data_dir + sep + "mouse";
	string model_dirR = data_dir + sep + "mouse";

	string save_file_B = output_dir + sep + "B.bmp";
	string save_file_pha_L = output_dir + sep + "phaL.bmp";
	string save_file_pha_R = output_dir + sep + "phaR.bmp";
	string save_file_pha_LC = output_dir + sep + "phaLC.bmp";
	string save_file_pha_RC = output_dir + sep + "phaRC.bmp";

	string disp8_img = output_dir + sep + "disp8.jpg";
	string dispRGB = output_dir + sep + "dispRGB.jpg";
	string disp_txt = output_dir + sep + "disp.txt";
	string point_cloud_txt = output_dir + sep + "point3D.xyz";

	string save_file_point3d = output_dir + sep + "xyz3.txt";

	string save_ply_point3d = output_dir + sep + "xyz.ply";


	//参数设置
	const int   Nphase = 9;           //多频相移法条纹图数目
	const int   HLratio = 4;          //高低频频率比

	const int   N = 3;                // 相移法：相移步数
	const int   n = 4;                // 格雷码：编码数量
	const int   n_wb = 2;             // 格雷码：额外编码

	const int   Hnum = 2;             // 横方向条纹像素
	const int   Vnum = 2;             // 竖方向条纹像素


	const float I_thr = 0.5;          // 格雷码：阈值
	const float B_min = 5;            // 最低调制度
	const int   winSize_filter = 3;   // 

	const int   winSize_match = 3;    //匹配窗口的大小
	const float pha_dif_value = 0.3;  //相位差的阈值，该阈值应该根据什么条件进行设置


	float min_z = 300, max_z = 1900;

	


};


//投影仪设置相关信息
struct PROJECTORPARA
{
	int Exptime ;     //曝光时间
	int Patperiod ;   //序列周期
	int Imgnum;       //投影图像数量
	bool Repeate;     //是否重复
	int hpierod = 20, vpierod = 10;
};



struct PROJECTORPARA2
{
	int Exptime;     //曝光时间
	int Patperiod;   //序列周期
	int Imgnum;       //投影图像数量
	bool Repeate;     //是否重复
	int hpierod = 20, vpierod = 10;
};



//标定相关的输入参数
struct CALIBRATION
{
	//行、列格子数目，值=总数目-1；格子的尺寸,单位mm
	int cols ;  //列
	int rows ;  //行
	double square_size;

	//图像的分辨率大小
	int W = 2048;  // 图片宽(单位pixel)
	int H = 1536;   // 图像高

	//图片保存的文件地址
	string sep = "/";
	string root_dir = R"(./)";
	string data_dir = root_dir + sep + "data";

	string data_calib = data_dir + "/calib";

	string calib_dirL = data_dir + sep + "calib/R";
	string calib_dirR = data_dir + sep + "calib/L";

	//保存相机标定的内参和外部参数
	string intrinsic_file = data_dir + sep + "intrinsic_file.yml";
	string extrinsic_file = data_dir + sep + "extrinsic_file.yml";
	string rectify_file = data_dir + sep + "rectify_file.yml";


	string  intrinsic_file1 = data_dir + sep + "intrinsic_file.txt";
	string  extrinsic_file1 = data_dir + sep + "extrinsic_file.txt";

	bool show = false;
};



struct PARA
{
	const int W = 2048;  // 图片宽(单位pixel)
	const int H = 1356;   // 图像高
	bool show = false;
};


struct CAMAPAPA
{
	const int OffX = 0;
	const int OffY = 0;
	const int MaxX = 2048;
	const int MaxY = 1356;
};


#endif //CONFIG_H