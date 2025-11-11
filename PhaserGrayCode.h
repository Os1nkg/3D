#ifndef PHASERGRAYCODE_H
#define PHASERGRAYCODE_H
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
using namespace std;
#define M_PI 3.1415926535897932


class PhaserGrayCode 
{

protected:
    
	int Nphase;   //相移图像总的数目
	int HLratio;  //频率比
	
	int N;       //相位移动数量
    int n;       //格雷码数量
    int n_wb;    //黑白图像
    float I_thr; //阈值


	float Hpixelnum;  //水平方向一个周期多少个条纹
	float Vpixelnum;  //竖直方向一个周期多少个条纹



    cv::Mat pha_wrapped, ks, ks2;
    cv::Size img_size;
	int m_h, m_w;


	//
	map<int, int> V2K;
	vector<int> vs_row;

	map<int, int> V2KN1;
	vector<int> vs_rowN1;


public:

	bool PHinit(int N, int Nphase,int HLratio);

    bool init(int N, int n, int n_wb, float I_thr);

	bool initCP(int N, int n, int n_wb, float I_thr, float Hnum, float Vnum);


    bool calcPhase(vector<cv::String> &files, cv::Mat &pha, cv::Mat &B);

	bool calcPhaseCPcalibration(vector<cv::String>& files, cv::Mat& phaH, cv::Mat& phaV, cv::Mat &Img);

	bool PhaseCPcalibration(vector<cv::Mat>& files, cv::Mat& pha);



	bool CPcalcPhase(vector<cv::String>& files, cv::Mat& phaH, cv::Mat& phaW);


	bool MultiScalePhase(vector<cv::String> &files, cv::Mat &pha, cv::Mat &B);

    bool filterGradient(cv::Mat &img_src, cv::Mat &img_des);

    bool filterB(cv::Mat &img_src, cv::Mat &img_des, cv::Mat &B, float B_min);

	bool phaseunwrap(vector<cv::Mat> &unwrapedphase, cv::Mat &wrapedphase);


protected:
    vector<int> makeGrayCode(int n);

    bool calc_phase_shift(vector<cv::Mat> &imgs, cv::Mat &pha, cv::Mat &GrayImg, cv::Mat &B, int N);

    bool calc_gray_code(vector<cv::Mat> &imgs, cv::Mat &ks, double I_thr, int n);

	bool calc_gray_codeN1(vector<cv::Mat> &imgs, cv::Mat &ks, double I_thr, int n);


    bool calc_gcs(vector<cv::Mat> &imgs, double I_thr, vector<cv::Mat> &gcs);


	bool Mycalc_gcs(vector<cv::Mat> &imgs, cv::Mat &GrayImg, vector<cv::Mat> &gcs);

	void calc_Multiscale_phase_shift(vector<cv::Mat> &imgs, vector<cv::Mat> &pha, cv::Mat &B, int N);

};

#endif 
