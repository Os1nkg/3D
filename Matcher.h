#ifndef MATCHER_H
#define MATCHER_H
#include <opencv2/opencv.hpp>
using namespace std;
class Matcher 
{
    cv::Size winSize;
    int wh;
    float pha_dif;

	int imgH;
	int imgW;

public:
    bool init(int win_size, float pha_dif);


	//bool phaseMatchPoint(cv::Mat &phaL, cv::Mat &phaR, vector<pair<cv::Point2f, cv::Point2f>> &cps);
    void phaseMatch(cv::Mat &phaL, cv::Mat &phaR, cv::Mat& depthmap);


	
	bool phaseMatchMy(cv::Mat &phaL, cv::Mat &phaR, cv::Mat& depthmap);

	bool phaseMatchMy2(cv::Mat& phaL, cv::Mat& phaR, cv::Mat& depthmap);

	bool phaseMatchMy3(cv::Mat& phaL, cv::Mat& phaR,  cv::Mat& depthmap);

	bool phaseMatchMy4(cv::Mat& phaL, cv::Mat& phaR, cv::Mat& depthmap);

	bool phaseMatchBest(cv::Mat& phaL, cv::Mat& phaR, vector<pair<cv::Point2f, cv::Point2f>>& cps);

	bool phaseMatchBest2(cv::Mat& phaL, cv::Mat& phaR, cv::Mat& dispmap);

	void find_featurepionts(cv::Mat& leftphase, cv::Mat& rightphase,
		vector<pair<cv::Point2f, cv::Point2f>> &cps);

	void find_featurepionts2(cv::Mat& leftphase, cv::Mat& rightphase,
		vector<pair<cv::Point2f, cv::Point2f>> &cps);

	void find_featurepionts_single_match(cv::Mat& leftphase, cv::Mat& rightphase,
		vector<pair<cv::Point2f, cv::Point2f>> &cps);

	void find_featurepionts_single_match2(cv::Mat& leftphase, cv::Mat& rightphase,
		vector<pair<cv::Point2f, cv::Point2f>> &cps);

	void featureMatch(cv::Mat& leftphase, cv::Mat& rightphase, cv::Mat& depthmap);


protected:
    bool is_valid_box(cv::Mat &BOX);

	int phase_searchbest(cv::Mat& BOX_L, cv::Mat& phaR, int Y_R, int X_R_Start);
	int phase_searchbest2(cv::Mat& BOX_L, cv::Mat& phaR, int Y_R, int X_R_Start,int X_End);

	bool is_valid_pixel(double l, double m, double r);



	int phase_search(cv::Mat& BOX_L, cv::Mat& phaR, int Y_R, int X_L, int X_R_Start);


	int phase_search1(double *phaL, double *phaR, int X_L, int X_R_Start,int XR_End);

	int phase_search2(double * phaL, double * phaR, int X_L, int X_R_Start, int XR_End);

    double calc_std(cv::Mat &box1, cv::Mat &box2);
    double search_sub_pixel(cv::Mat &BOX_L, cv::Mat &phaR, int XR, int YR);
};


#endif 
