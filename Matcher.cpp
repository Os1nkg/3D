#include "Matcher.h"
#include "Spline.h"


bool Matcher::init(int win_size, float pha_dif)
{
    winSize = cv::Size (win_size, win_size); //窗口的大小
    wh = win_size / 2;  //1
    this->pha_dif = pha_dif;
    return false;
}


static void saveTXTS(const char* filename, cv::Mat& mat)
{
	FILE* fp = fopen(filename, "wt"); //"wt" 只写打开或建立一个文本文件，只读写数据 （t为文本文件)
	printf("%d %d \n", mat.rows, mat.cols);
	for (int y = 0; y < mat.rows; y++)
	{
		short* disp = mat.ptr<short>(y);
		for (int x = 0; x < mat.cols; x++)
		{
			//float point = mat.at<double>(y, x);
			short point = disp[x];
			//cout << point << std::endl;

			fprintf(fp, "%d ", point);
		}
		fprintf(fp, "\n");
		fprintf(fp, "\n");
	}
	fclose(fp);
}

void Matcher::featureMatch(cv::Mat& leftphase, cv::Mat& rightphase,cv::Mat& depthmap)
{
	int H = leftphase.rows;
	int W = leftphase.cols;
	//int x, y, k1, k2;
	float left, right;
	cv::Point2f fleft, fright;
	float* pre_right_data;
	int pre_k;


	for (int y = 0; y < H; y++)
	{
		float* left_phase_data = leftphase.ptr<float>(y);
		float* right_phase_data = rightphase.ptr<float>(y);
		float* depth = depthmap.ptr<float>(y);

		float minphase = 1000.0f;
		int k1 = 0;

		for (int x = 0; x < W; x++)
		{
			float left = left_phase_data[x];  //左图像中单个像素值

			for (int xr = 0; xr < W; xr++)
			{
				abs(left_phase_data[x] - right_phase_data[xr]) < minphase;  //记录下最小相位差的点
				k1 = xr;
			}
		    

			depth[x] = abs(x - k1);

			
		}
	}
}


//这种方法可以采用，比较好的一种方法
void Matcher::find_featurepionts(cv::Mat& leftphase, cv::Mat& rightphase,
	vector<pair<cv::Point2f, cv::Point2f>> &cps)
{
	int nr = leftphase.rows;
	int nc = leftphase.cols;
	int x, y, k1, k2;
	float left, right;
	cv::Point2f fleft, fright;
	float *pre_right_data;
	int pre_k;


	for (y = 0; y < nr; y ++)
	{
		float *left_phase_data = leftphase.ptr<float>(y);
		float *right_phase_data = rightphase.ptr<float>(y);
		float *left_phase_data2;
		
		for (x = 0; x < nc; x++)
		{
			left = *left_phase_data++;  //左图像中单个像素值

				k1 = 0;

				//找到两者绝对值小于一定数值的点
				while ((abs(left - *right_phase_data++) > pha_dif) && (k1 < nc)) k1++;



				if (k1 < nc)
				{
					
					right = *(--right_phase_data);
					left_phase_data2 = leftphase.ptr<float>(y);
					k2 = 0;
					while ((abs(right - *left_phase_data2++) > pha_dif) && (k2 < nc)) k2++;

					if ((k2 < nc) && (abs(k2 - x) < 2))
					{
						fleft.x = (x + k2) / 2;
						fleft.y = y;
						fright.x = k1;
						fright.y = y;
						//leftkeypoint.push_back(fleft);
						//rightkeypoint.push_back(fright);
						pair<cv::Point2f, cv::Point2f> cp(fleft, fright);
						cps.emplace_back(cp);

					}
				}
			

		}
	}
}



//这种方法可以采用，比较好的一种方法
void Matcher::find_featurepionts2(cv::Mat& leftphase, cv::Mat& rightphase,
	vector<pair<cv::Point2f, cv::Point2f>> &cps)
{
	int nr = leftphase.rows;
	int nc = leftphase.cols;
	int x, y, k1, k2;
	float left, right;
	cv::Point2f fleft, fright;
	float *pre_right_data;
	int pre_k;

     //#pragma omp parallel for
	for (y = 0; y < nr; y++)
	{
		float *left_phase_data = leftphase.ptr<float>(y);
		float *right_phase_data = rightphase.ptr<float>(y);
		float *left_phase_data2 = leftphase.ptr<float>(y);

		for (x = 0; x < nc-1; x++)
		{
			left = *left_phase_data++;
			k1 = 1;
			while (((left - right_phase_data[k1-1])*(left - right_phase_data[k1 + 1]) > 0.0f) && (k1 < nc-1)) k1++;

			if (k1 < nc-1)
			{
				right = right_phase_data[k1];
				//left_phase_data2 = leftphase.ptr<float>(y);
				k2 = 1;
				while (((right - left_phase_data2[k2-1])*(right - left_phase_data2[k2+1]) > 0.0f) && (k2 < nc-1)) k2++;

				if ((k2 < nc) && (abs(k2 - x) < 2))
				{
					fleft.x = (x + k2) / 2;
					fleft.y = y;
					fright.x = k1;
					fright.y = y;
					//leftkeypoint.push_back(fleft);
					//rightkeypoint.push_back(fright);
					pair<cv::Point2f, cv::Point2f> cp(fleft, fright);
			    	cps.emplace_back(cp);

				}
			}


		}
	}
}



//只用左边的匹配右边的，这样节省一半的时间。采用这两种方法
void Matcher::find_featurepionts_single_match(cv::Mat& leftphase, cv::Mat& rightphase,
	vector<pair<cv::Point2f, cv::Point2f>> &cps)
{
	int nr = leftphase.rows;
	int nc = leftphase.cols;
	int x, y, k;
	float left;
	cv::Point2f fleft, fright;

	for (y = 0; y < nr; y += 1)
	{
		float *left_phase_data = leftphase.ptr<float>(y);
		float *right_phase_data = rightphase.ptr<float>(y);

		for (x = 0; x < nc; x++)
		{
			left = *left_phase_data++;

			if (left > 2 * CV_PI)
			{
				right_phase_data = rightphase.ptr<float>(y);
				k = 0;

				while ((abs(left - *right_phase_data++) > pha_dif) && (k < nc)) k++;
				if (k < nc)
				{
					fleft.x = x;
					fleft.y = y;
					fright.x = k;
					fright.y = y;
					//leftkeypoint.push_back(fleft);
					//rightkeypoint.push_back(fright);
					pair<cv::Point2f, cv::Point2f> cp(fleft, fright);
					cps.emplace_back(cp);
				}
			}

		}
	}
}



//只用左边的匹配右边的，这样节省一半的时间。采用这两种方法
void Matcher::find_featurepionts_single_match2(cv::Mat& leftphase, cv::Mat& rightphase,
	vector<pair<cv::Point2f, cv::Point2f>> &cps)
{
	int nr = leftphase.rows;
	int nc = leftphase.cols;
	int x, y, k;
	float left;
	cv::Point2f fleft, fright;

	for (y = 0; y < nr; y += 1)
	{
		float *left_phase_data = leftphase.ptr<float>(y);
		float *right_phase_data = rightphase.ptr<float>(y);

		for (x = 0; x < nc; x++)
		{
			left = *left_phase_data++;

			
				//right_phase_data = rightphase.ptr<float>(y);
				k = 1;

				while (((left - right_phase_data[k-1])*(left - right_phase_data[k+1]) > 0) && (k < nc-1)) k++;


				if (k < nc)
				{
					fleft.x = x;
					fleft.y = y;
					fright.x = k;
					fright.y = y;
					//leftkeypoint.push_back(fleft);
					//rightkeypoint.push_back(fright);

					pair<cv::Point2f, cv::Point2f> cp(fleft, fright);
					cps.emplace_back(cp);
				}
			

		}
	}
}



int Matcher::phase_searchbest(cv::Mat& BOX_L, cv::Mat& phaR, int Y_R, int X_R_Start)
{

	//中心点的相位值
	double l = BOX_L.at<double>(wh, wh);
	vector<pair<cv::Mat, int>> ps;

	double* Rpha = phaR.ptr<double>(Y_R);
	// 01 在该行搜索有效匹配框
	// #pragma omp parallel for
	for (int X_R = X_R_Start; X_R < phaR.cols - wh; X_R++)
	{
		
		//目标点的相位值
		double r = Rpha[X_R];

		// 如果差值小于阈值，并且为有效搜索框，加入到候选匹配框中
		if (abs(l - r) <= pha_dif)
		{
			cv::Rect roi_R(cv::Point(X_R - wh, Y_R - wh), winSize);
			cv::Mat BOX_R = phaR(roi_R);
			if (is_valid_box(BOX_R))
			{
				pair<cv::Mat, int> p(BOX_R, X_R);
				ps.emplace_back(p);
			}

		}
	}
	// 如果为空，那么返回False
	if (ps.empty()) {
		return -1;
	}

	// 02 计算最佳匹配点
	double min_v = 10000;
	int XR = -1;


	//#pragma omp parallel for
	for (auto& p : ps)
	{
		double v = calc_std(BOX_L, p.first);
		if (v < min_v) {
			min_v = v;
			XR = p.second;
		}
	}

	return XR;
}



int Matcher::phase_searchbest2(cv::Mat& BOX_L, cv::Mat& phaR, int Y_R, int X_R_Start,int X_End)
{

	//中心点的相位值
	double l = BOX_L.at<double>(wh, wh);
	vector<pair<cv::Mat, int>> ps;

	double* Rpha = phaR.ptr<double>(Y_R);
	// 01 在该行搜索有效匹配框
	// #pragma omp parallel for
	for (int X_R = X_R_Start; X_R < X_End; X_R++)
	{
		
		//目标点的相位值
		double r = Rpha[X_R];

		// 如果差值小于阈值，并且为有效搜索框，加入到候选匹配框中
		if (abs(l - r) <= pha_dif)
		{
			cv::Rect roi_R(cv::Point(X_R - wh, Y_R - wh), winSize);
			cv::Mat BOX_R = phaR(roi_R);
			if (is_valid_box(BOX_R))
			{
				pair<cv::Mat, int> p(BOX_R, X_R);
				ps.emplace_back(p);
			}

		}
	}
	// 如果为空，那么返回False
	if (ps.empty()) {
		return -1;
	}

	// 02 计算最佳匹配点
	double min_v = 10000;
	int XR = -1;


	//#pragma omp parallel for
	for (auto& p : ps)
	{
		double v = calc_std(BOX_L, p.first);
		if (v < min_v) {
			min_v = v;
			XR = p.second;
		}
	}

	return XR;
}




// 进行立体相位匹配
bool Matcher::phaseMatchBest(cv::Mat& phaL, cv::Mat& phaR, vector<pair<cv::Point2f, cv::Point2f>>& cps)
{
	

	//
	//#pragma omp parallel for
	for (int Y_L = wh; Y_L < phaL.rows - wh; Y_L++)
	{
		bool flag_glob_search = true;
		int XR;
		bool flag_find;

		for (int X_L = wh; X_L < phaL.cols - wh; X_L++)
		{
			//((x,y),(w,h))一个小的矩形框
			cv::Rect roi_L(cv::Point(X_L - wh, Y_L - wh), winSize);

			cv::Mat BOX_L = phaL(roi_L);   //得到小块矩形区域内的相位值
			flag_find = false; // 默认没找到
			if (is_valid_box(BOX_L))
			{
				// 02 默认进行全局搜索
				if (flag_glob_search)
				{
					//进行匹配操作
					XR = phase_searchbest(BOX_L, phaR, Y_L, wh);
					XR == -1 ? flag_find = false : flag_find = true;
					if (flag_find)
					{
						flag_glob_search = false;
					}
				}
				else {
					// 04 局部搜索
					int X_R_PRE = XR;
					XR = phase_searchbest(BOX_L, phaR, Y_L, X_R_PRE);
					XR == -1 ? flag_find = false : flag_find = true;
					// 05 如果局部没找到，那么下个像素又开始全局搜索
					if (!flag_find) {
						flag_glob_search = true;
					}
				}
			}


			// 06 亚像素插值、添加搜索结果
			//只有找到匹配点，才进行保存，否则应该进行处理
			if (flag_find)
			{
				//搜索亚像素位置
				//double XR_new = search_sub_pixel(BOX_L, phaR, XR, Y_L);

				cv::Point2f p_L, p_R;
				p_L.x = float(X_L); p_L.y = float(Y_L);
				//p_R.x = float(XR_new); p_R.y = float(Y_L);
				p_R.x = float(XR); p_R.y = float(Y_L);

				pair<cv::Point2f, cv::Point2f> cp(p_L, p_R);
				cps.emplace_back(cp);
			}


		}
	}
	cout << "匹配点对数:" << cps.size() << std::endl;
	return !cps.empty();
}




// 进行立体相位匹配
bool Matcher::phaseMatchBest2(cv::Mat& phaL, cv::Mat& phaR, cv::Mat& dispmap)
{
	//
	#pragma omp parallel for
	for (int Y_L = wh; Y_L < phaL.rows - wh; Y_L++)
	{
		bool flag_glob_search = true;
		int XR;
		bool flag_find;
		float* distemp = dispmap.ptr<float>(Y_L);
		for (int X_L = wh; X_L < phaL.cols - wh; X_L++)
		{
			//((x,y),(w,h))一个小的矩形框
			cv::Rect roi_L(cv::Point(X_L - wh, Y_L - wh), winSize);
			cv::Mat BOX_L = phaL(roi_L);   //得到小块矩形区域内的相位值

			flag_find = false; // 默认没找到
			if (is_valid_box(BOX_L))
			{
				// 02 默认进行全局搜索
				if (flag_glob_search)
				{
					//进行匹配操作
					XR = phase_searchbest2(BOX_L, phaR, Y_L, wh,X_L);
					XR == -1 ? flag_find = false : flag_find = true;
					if (flag_find)
					{
						flag_glob_search = false;
					}
				}
				else 
				{
					// 04 局部搜索
					int X_R_PRE = XR;
					XR = phase_searchbest2(BOX_L, phaR, Y_L, X_R_PRE,X_L);
					XR == -1 ? flag_find = false : flag_find = true;
					// 05 如果局部没找到，那么下个像素又开始全局搜索
					if (!flag_find) 
					{
						flag_glob_search = true;
					}
				}
			}

			// 06 亚像素插值、添加搜索结果
			//只有找到匹配点，才进行保存，否则应该进行处理
			if (flag_find)
			{
				distemp[X_L] = X_L - XR;
				if (distemp[X_L] < 0)
					distemp[X_L] = 0;
			}



		}
	}
	//cout << "匹配点对数:" << cps.size() << std::endl;
	//return !cps.empty();
	return true;
}



bool Matcher::phaseMatchMy2(cv::Mat& phaL, cv::Mat& phaR, cv::Mat& depthmap)
{

    #pragma omp parallel for
	for (int Y_L = 0; Y_L < phaL.rows ; Y_L++)
	{
		int num = 0;
		int XR;
		bool flag_find;

		
		double * pL = phaL.ptr<double>(Y_L);
		double * pR = phaR.ptr<double>(Y_L);
		short * disp = depthmap.ptr<short>(Y_L);

		for (int X_L = wh; X_L < phaL.cols - wh; X_L++)
		{

			flag_find = false; // 默认没找到



			if (pL[X_L]>0)
			{
				// 02 默认进行全局搜索
				
				XR = phase_search2(pL, pR, X_L,wh, X_L);   //进行全局匹配操作
				
				XR == -1 ? flag_find = false : flag_find = true;
			

				if (flag_find)
				{
					disp[X_L] = X_L - XR;
					if (disp[X_L] < 0)
						disp[X_L] = 0;
					
				}


			}
		}
	}

	
	return true;

}




bool Matcher::phaseMatchMy3(cv::Mat& phaL, cv::Mat& phaR, cv::Mat& depthmap)
{

	cv::Mat dispL = cv::Mat::zeros(phaL.size(), CV_16SC1);
	cv::Mat dispR = cv::Mat::zeros(phaR.size(), CV_16SC1);
	cv::Mat Temp = cv::Mat::zeros(phaR.size(), CV_16SC1);
	cv::Mat Temp1 = cv::Mat::zeros(phaR.size(), CV_16SC1);

    #pragma omp parallel for
	for (int Y_L = 0; Y_L < phaL.rows; Y_L++)
	{
		int num = 0;
		int XR;
		bool flag_find;

		double * pL = phaL.ptr<double>(Y_L);
		double* pR = phaR.ptr<double>(Y_L);

		//float* bL = BL.ptr<float>(Y_L);
		//float* bR = BR.ptr<float>(Y_L);

		short * disL = dispL.ptr<short>(Y_L);
		short * disR = dispR.ptr<short>(Y_L);

		//左视差图计算
		for (int X_L = 1; X_L < phaL.cols-1; X_L++)
		{
			flag_find = false; // 默认没找到
			int minpos = 0;
			double l = pL[X_L];
			double l1 = pL[X_L - 1];
			double l2 = pL[X_L +1];

			if (l >pha_dif&&l1> pha_dif&&l2> pha_dif)
			{

				for (int X_R = 1; X_R < X_L; X_R++)
				{
					double r = pR[X_R];

					if (r > pha_dif)
					{

						// 如果差值小于阈值，并且为有效搜索框，加入到候选匹配框中
						if (abs(l - r) <= pha_dif)
						{

							float lz = (l1 - r) * (l2 - r);
							//float rz = (pR[X_R - 1] - pL[X_L]) * (pR[X_R + 1] - pL[X_L]);
							//if (lz < 0 && rz < 0)
							if (lz < 0)
							{
								minpos = X_R;
								flag_find = true;
							}
						}
					}
				}
			}

			if (flag_find)
				disL[X_L] = X_L - minpos;
			else
				disL[X_L] = 0;

		}

		//右视差图计算
		for (int X_R = 1; X_R < phaR.cols - 1; X_R++)
		{
			flag_find = false; // 默认没找到
			int minpos = 0;

			double r= pR[X_R];
			double r1 = pR[X_R-1];
			double r2 = pR[X_R+1];

			if (r > pha_dif&& r1 > pha_dif&& r1 > pha_dif)
			{
				for (int X_L = X_R; X_L < phaL.cols - 1; X_L++)
				{

					double  l = pL[X_L];
					// 如果差值小于阈值，并且为有效搜索框，加入到候选匹配框中
					if (l > pha_dif)
					{
						if (abs(l - r) <= pha_dif)
						{
							//float lz = (pL[X_L - 1] - pR[X_R]) * (pL[X_L + 1] - pR[X_R]);
							float rz = (r1 - l) * (r2 - l);
							//if (lz < 0 && rz < 0)
							if (rz < 0)
							{
								minpos = X_L;   //
								flag_find = true;
							}
						}
					}
				}
			}


			if(flag_find)
			    disR[X_R] = minpos-X_R;
			else 
				disR[X_R] = 0;

		}
		//
	}


	cv::medianBlur(dispL,dispL, 5);
	cv::medianBlur(dispR, dispR, 5);

	//进行左右一致性检测
    #pragma omp parallel for
	for (int y = 0; y < phaL.rows; y++)
	{
		short * depth= Temp.ptr<short>(y);
		short * pL = dispL.ptr<short>(y);
		short * pR = dispR.ptr<short>(y);
		int xr = 0;
		for (int x = 0; x < phaL.cols; x++)
		{
			short dl = pL[x];
			xr = x - dl;
			if (xr < 0)
				xr = 0;

			short dr = pR[xr];

			if (dl > 10 && abs(int(dr - dl)) < 2)
				depth[x] = dl;
			else
				depth[x] = 0;
		}
	}

    #pragma omp parallel for
	for (int y = 0; y < phaL.rows; y++)
	{
		short* depth = Temp1.ptr<short>(y);
		short* pL = dispL.ptr<short>(y);
		short* pR = dispR.ptr<short>(y);
		int xl = 0;
		for (int x = 0; x < phaL.cols; x++)
		{
			short dr = pR[x];
			xl = x + dr;
			
			if (xl > phaL.cols - 1)
				xl = phaL.cols - 1;

			short dl = pL[xl];

			//if (dl > 10 || abs(int(dr - dl)) < 2)
			if (dl > 10 && abs(int(dr - dl)) < 2)
				depth[x] = dr;
			else
				depth[x] = 0;
		}
	}


	cv::medianBlur(Temp, depthmap, 5);
	

	cv::imwrite("DispR.png", dispR);
	cv::imwrite("DispL.png", dispL);
	cv::imwrite("DispLRefined.png", depthmap);
	cv::imwrite("DispRRefined.png", Temp1);

	return true;

}


bool Matcher::phaseMatchMy4(cv::Mat& phaL, cv::Mat& phaR, cv::Mat& depthmap)
{

	cv::Mat dispL = cv::Mat::zeros(phaL.size(), CV_16SC1);
	cv::Mat dispR = cv::Mat::zeros(phaR.size(), CV_16SC1);

    #pragma omp parallel for
	for (int Y_L = 0; Y_L < phaL.rows; Y_L++)
	{
		int num = 0;
		int XR;
		bool flag_find;

		//short* depth = depthmap.ptr<short>(Y_L);

		double* pL = phaL.ptr<double>(Y_L);
		double* pR = phaR.ptr<double>(Y_L);

		//float* bL = BL.ptr<float>(Y_L);
		//float* bR = BR.ptr<float>(Y_L);

		short* disL = dispL.ptr<short>(Y_L);
		short* disR = dispR.ptr<short>(Y_L);

		//左视差图计算
		for (int X_L = 1; X_L < phaL.cols - 1; X_L++)
		{
			flag_find = false; // 默认没找到
			short minpos = 0;
			double l = pL[X_L];
			double temp = 0;
			double minpha = 1000.0f;
			if (l > pha_dif)
			{

				for (int X_R = 1; X_R < X_L; X_R++)
				{
					double r = pR[X_R];

					if (r > pha_dif)
					{

						// 如果差值小于阈值，并且为有效搜索框，加入到候选匹配框中
						if (abs(l - r) <= pha_dif)
						{

							//float lz = (pL[X_L - 1] - pR[X_R]) * (pL[X_L + 1] - pR[X_R]);
							//float rz = (pR[X_R - 1] - pL[X_L]) * (pR[X_R + 1] - pL[X_L]);
							//if (lz < 0 && rz < 0)
							temp = (pL[X_L - 1] - pR[X_R - 1]) * (pL[X_L - 1] - pR[X_R - 1]) + (pL[X_L] - pR[X_R]) * (pL[X_L] - pR[X_R])
								+ (pL[X_L + 1] - pR[X_R + 1]) * (pL[X_L + 1] - pR[X_R + 1]);
							temp = sqrt(temp);

							if (minpha > temp)
							{
								minpha = temp;
								minpos = X_R;
								flag_find = true;
							}
						}
					}
				}
			}

			if (flag_find)
				disL[X_L] = short(X_L - minpos);
			else
				disL[X_L] = 0;

		}

		//右视差图计算
		for (int X_R = 1; X_R < phaR.cols - 1; X_R++)
		{
			flag_find = false; // 默认没找到
			int minpos = 0;
			double temp = 0;
			double minpha = 1000.0f;
			double r = pR[X_R];

			if (r > pha_dif)
			{
				for (int X_L = X_R; X_L < phaL.cols - 1; X_L++)
				{

					double  l = pL[X_L];
					// 如果差值小于阈值，并且为有效搜索框，加入到候选匹配框中
					if (l > pha_dif)
					{
						if (abs(l - r) <= pha_dif)
						{
							//float lz = (pL[X_L - 1] - pR[X_R]) * (pL[X_L + 1] - pR[X_R]);
							//float rz = (pR[X_R - 1] - pL[X_L]) * (pR[X_R + 1] - pL[X_L]);
							//if (lz < 0 && rz < 0)
							temp = (pL[X_L - 1] - pR[X_R - 1]) * (pL[X_L - 1] - pR[X_R - 1]) + (pL[X_L] - pR[X_R]) * (pL[X_L] - pR[X_R])
								+ (pL[X_L + 1] - pR[X_R + 1]) * (pL[X_L + 1] - pR[X_R + 1]);
							temp = sqrt(temp);

							if (minpha > temp)
							{
								minpha = temp;
								minpos = X_L;
								flag_find = true;
							}

						}
					}
				}
			}


			if (flag_find)
				disR[X_R] = short(minpos - X_R);
			else
				disR[X_R] = 0;

		}

		/*
		for (int x = 0; x < phaL.cols; x++)
		{
			short dl = pL[x];
			int xr = x - dl;
			if (xr < 0)
				xr = 0;
			if (xr >= phaL.cols)
				xr = phaL.cols - 1;

			short dr = pR[xr];

			if (dl > 2 && abs((dr - dl)) < 2)
				depth[x] = dl;
			else
				depth[x] = 0;
		}
		*/



	}


	//saveTXTS("DispL.txt", dispL);
	//saveTXTS("DispR.txt", dispR);

	//cv::Mat dL = dispL / 257;
	//dL.convertTo(dL, CV_8U);
	//cv::Mat dR = dispR / 257;
	//dR.convertTo(dR, CV_8U);

    #pragma omp parallel for
	for (int y = 0; y < phaL.rows; y++)
	{
		short* dep = depthmap.ptr<short>(y);
		short* pL = dispL.ptr<short>(y);
		short* pR = dispR.ptr<short>(y);
		for (int x = 0; x < phaL.cols; x++)
		{
			short dl = pL[x];
			int xr = x - dl;
			if (xr < 0 )
				xr = 0;
			if (xr >= phaL.cols)
				xr = phaL.cols - 1;

			short dr = pR[xr];

			if (dl > 5 && abs(int(dr - dl)) <=1)
				dep[x] = (short)dl;
			else
				dep[x] = 0;
		}
	}

	cv::imwrite("DispR.png", dispR);
	cv::imwrite("DispL.png", dispL);
	

	cv::imwrite("DispLRefined.png", depthmap);

	return true;

}




// 进行立体相位匹配
bool Matcher::phaseMatchMy(cv::Mat &phaL, cv::Mat &phaR, cv::Mat& dispmap)
{

	#pragma omp parallel for
	for (int Y_L = wh; Y_L < phaL.rows - wh; Y_L++)
	{
		bool flag_glob_search = true;
		int num = 0;
		int XR=0;
		bool flag_find=false;

		short * distemp = dispmap.ptr<short>(Y_L);
		double * pL = phaL.ptr<double>(Y_L);
		double * pR = phaR.ptr<double>(Y_L);

		for (int X_L = wh; X_L < phaL.cols - wh; X_L++)
		{

			flag_find = false; // 默认没找到
			if (is_valid_pixel(pL[X_L-1],pL[X_L],pL[X_L+1]))
			{
				// 02 默认进行全局搜索
				if (flag_glob_search)
				{
					XR = phase_search1(pL,pR,X_L, wh,X_L);   //进行全局匹配操作
					XR == -1 ? flag_find = false : flag_find = true;
					if (flag_find)
					{
						flag_glob_search = false;
					}
				}
				else
				{
					// 04 局部搜索
					int X_R_PRE = XR;
					XR = phase_search1(pL, pR, X_L, wh,X_L);
					XR == -1 ? flag_find = false : flag_find = true;
					// 05 如果局部没找到，那么下个像素又开始全局搜索
					if (!flag_find) 
					{
						flag_glob_search = true;
					}
				}
			}

			// 06 亚像素插值、添加搜索结果
			if (flag_find)
			{
				distemp[X_L] = X_L-XR;
				if (distemp[X_L] < 0)
					distemp[X_L] = 0;
			}

		}




	}
	
}




int Matcher::phase_search2(double * pL, double * pR, int X_L, int X_R_Start, int XR_End)
{

	//中心点的相位值
	vector<int> ps;
	double l = pL[X_L];

	float minpha = 10000.0f;
	int minpos = -1;
	float temp = 0.0f;
	// 01 在该行搜索有效匹配框

	for (int X_R = X_R_Start; X_R < XR_End; X_R++)
	{
		double r = pR[X_R];
		// 如果差值小于阈值，并且为有效搜索框，加入到候选匹配框中
		if (abs(l - r) <= pha_dif && r>0.0f)
		{
			float lz = (pL[X_L - 1] - pR[X_R]) * (pL[X_L + 1] - pR[X_R]);
			float rz = (pR[X_R - 1] - pL[X_L]) * (pR[X_R + 1] - pL[X_L]);

			if (lz<0&&rz<0)
			{
				minpos = X_R;

				ps.push_back(X_R);
			}
		}
	}



	if (ps.empty()) {
		return -1;
	}

	// 02 计算最佳匹配点
	
	int num = ps.size();

	for (int i=0;i<num;i++)
	{
		int XR = ps[i];

		temp = (pL[X_L - 1] - pR[XR - 1]) * (pL[X_L - 1] - pR[XR - 1]) + (pL[X_L] - pR[XR]) * (pL[X_L] - pR[XR])
			+ (pL[X_L + 1] - pR[XR + 1]) * (pL[X_L + 1] - pR[XR + 1]);

		temp = sqrt(temp);

		if (minpha > temp)
		{
			minpha = temp;
			minpos = XR;
		}
	}


	return minpos;
}


int Matcher::phase_search1(double *pL, double *pR, int X_L, int X_R_Start,int XR_End)
{

	//中心点的相位值
	
	double l = pL[X_L];

	float minpha = 10000.0f;
	int minpos = -1;
	float temp = 0.0f;
	// 01 在该行搜索有效匹配框
	
	for (int X_R = X_R_Start; X_R<XR_End; X_R++)
	{
		double r = pR[X_R];
		// 如果差值小于阈值，并且为有效搜索框，加入到候选匹配框中
		if (abs(l - r) <= pha_dif)
		{
			
			if (is_valid_pixel(pR[X_R-1],pR[X_R], pR[X_R+1]))
			{
				
				temp = (pL[X_L - 1] - pR[X_R - 1]) * (pL[X_L - 1] - pR[X_R - 1]) + (pL[X_L] - pR[X_R]) * (pL[X_L] - pR[X_R])
					+ (pL[X_L + 1] - pR[X_R + 1]) * (pL[X_L + 1] - pR[X_R + 1]);
				temp = sqrt(temp);

				if (minpha > temp)
				{
					minpha = temp;
					minpos = X_R;
				}
			}
		}
	}
	
	return minpos;
}



// 进行立体相位匹配
void Matcher::phaseMatch(cv::Mat &phaL, cv::Mat &phaR, cv::Mat& depthmap)
{
    int XR;
    bool flag_find;

	//
	int num = 0;
    //#pragma omp parallel for
    for (int Y_L = wh; Y_L < phaL.rows - wh; Y_L++) 
	{
        bool flag_glob_search = true;
		float* depth = depthmap.ptr<float>(Y_L);

        for (int X_L = wh; X_L < phaL.cols - wh; X_L++) 
		{
			//((x,y),(w,h))一个小的矩形框
            cv:: Rect roi_L(cv::Point(X_L - wh, Y_L - wh), winSize);

            cv::Mat BOX_L = phaL(roi_L);   //得到小块矩形区域内的相位值
            flag_find = false; // 默认没找到
            if (is_valid_box(BOX_L))
			{
                // 02 默认进行全局搜索
                if (flag_glob_search)
				{
                    XR = phase_search(BOX_L, phaR, Y_L,X_L, X_L);   //进行全局匹配操作
                    XR == -1 ? flag_find = false : flag_find= true;
                    if (flag_find)
					{
                        flag_glob_search = false;
                    }
                }
                else
				{
                    // 04 局部搜索
                    int X_R_PRE = XR+2;
                    XR = phase_search(BOX_L, phaR, Y_L, X_L, X_R_PRE);
                    XR == -1 ? flag_find = false : flag_find= true;
                    // 05 如果局部没找到，那么下个像素又开始全局搜索
                    if (!flag_find){
                        flag_glob_search = true;
                    }
                }



            }

            // 06 亚像素插值、添加搜索结果
            if (flag_find)
			{
				depth[X_L] = X_L-XR;
				if (depth[X_L] < 0)
					depth[X_L] = 0;
				
            }
        }
    }

    cout << "匹配点对数:" << num<< std::endl;
}



int Matcher::phase_search(cv::Mat &BOX_L, cv::Mat &phaR, int Y_R, int X_L, int X_R_Start) 
{
    
	//中心点的相位值
	double l = BOX_L.at<double>(wh, wh);
    vector<pair<cv::Mat, int>> ps;
    
	// 01 在该行搜索有效匹配框
    // #pragma omp parallel for
    for (int X_R = X_R_Start; X_R >wh; X_R--) 
	{
		
		//目标点的相位值
        double r = phaR.at<double>(Y_R, X_R);
        
		// 如果差值小于阈值，并且为有效搜索框，加入到候选匹配框中
        if (abs(l - r) <= pha_dif)
		{
            cv::Rect roi_R(cv::Point(X_R - wh, Y_R - wh), winSize);
            cv::Mat BOX_R = phaR(roi_R);
            if (is_valid_box(BOX_R))
			{
                pair<cv::Mat, int> p(BOX_R, X_R);
                ps.emplace_back(p);
            }

        }
    }
    // 如果为空，那么返回False
    if (ps.empty()){
        return -1;
    }

    // 02 计算最佳匹配点
    double min_v = 1000;
    int XR = -1;

    //#pragma omp parallel for
    for (auto &p: ps)
	{
        double v = calc_std(BOX_L, p.first);
        if (v < min_v){
            min_v = v;
            XR = p.second;
        }
    }

    return XR;
}

// 如果相位值中存在零点，说明它无相位信息
// 或者在边缘，这些相位信息不可靠，不需要寻找
bool Matcher::is_valid_pixel(double l, double m, double r)
{
		if (l <= 0)
			return false;
		if (m <= 0)
			return false;
		if (r <= 0)
			return false;
	
	return true;
}


bool Matcher::is_valid_box(cv::Mat &BOX) 
{
    double min_v, max_v;
    cv::Point min_pt, max_pt;
    cv::minMaxLoc(BOX, &min_v, &max_v, &min_pt, &max_pt);
    if (min_v <= 0){
        return false;
    }
    return true;
}

//计算两个区域的之间的距离
double Matcher::calc_std(cv::Mat &box1, cv::Mat &box2)
{
    cv::Mat v_mat;
    cv::pow(box2 - box1, 2, v_mat);
    return sqrt(cv::sum(v_mat).val[0]);
}


//搜索亚像素位置
double Matcher::search_sub_pixel(cv::Mat &BOX_L, cv::Mat &phaR, int XR, int YR) 
{
    // 01 检查边界是否越界，越界的话，直接返回原始值即可
    if (XR - 2 < 0 || XR + 2 > phaR.cols)
	{
        return double(XR);
    }

    double xs[5], ys[5];
    int idx = 0;
    for (int i = -2; i <= 2; ++i)
	{   //该点位置的相位
        double pha_v = phaR.at<double>(YR, XR + i);
        // 如果存在0值，那么就不进行插值，直接返回
        if (pha_v == 0)
		{
            return double(XR);
        }

        xs[idx] = pha_v;           // 相位
        ys[idx] = double(XR + i);  // 坐标
        idx += 1;
    }


    SplineSpace::SplineInterface *sp = new SplineSpace::Spline(xs, ys, 5);
    double x = BOX_L.at<double>(wh, wh);
    // 范围内才进行数值上的插值
    if (x >= *min_element(&xs[0], &xs[4]) && x <= *max_element(&xs[0], &xs[4]))
	{
        double y;
        sp->SinglePointInterp(x, y);
        return y;
    }
    else
	{
        cv::Mat temp;
        cv::Rect ROI_R(cv::Point(XR, YR), winSize);
        cv::Mat BOX_R = phaR(ROI_R);
        cv::resize(BOX_R, temp, cv::Size(10, 9), 0, 0, cv::INTER_LINEAR);
        double min_val = 10000;
        double pos = 0;
        double sep = 3. / 10;
        for (int c = 0; c < temp.cols - 2; ++c)
		{
            cv::Rect roi(cv::Point(c, 3), winSize);
            cv::Mat BOX_R_temp = temp(roi);
            double val = calc_std(BOX_L, BOX_R_temp);
            if (val < min_val)
			{
                min_val = val;
                pos = sep + sep * c;
            }
        }
        return XR - 3. / 2 + pos ;
    }


}