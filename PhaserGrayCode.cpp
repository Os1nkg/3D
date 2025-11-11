#include "PhaserGrayCode.h"
#include "Myutils.h"
#include <cmath>
#include <map>


static void saveTXT(const char* filename, const Mat& mat)
{
	FILE* fp = fopen(filename, "wt");          //"wt" 只写打开或建立一个文本文件，只读写数据 （t为文本文件)
	printf("%d %d \n", mat.rows, mat.cols);
	for (int y = 0; y < mat.rows; y++)
	{
		for (int x = 0; x < mat.cols; x++)
		{
			int point = mat.at<double>(y, x);
			//cout << point << std::endl;

			fprintf(fp, "%d ", point);
		}
		fprintf(fp, "\n");
		fprintf(fp, "\n");
	}
	fclose(fp);
}


static void saveTXTFF(const char* filename, const Mat& mat)
{
	FILE* fp = fopen(filename, "wt"); //"wt" 只写打开或建立一个文本文件，只读写数据 （t为文本文件)
	printf("%d %d \n", mat.rows, mat.cols);
	for (int y = 0; y < mat.rows; y++)
	{
		for (int x = 0; x < mat.cols; x++)
		{
			float point = mat.at<double>(y, x);
			//cout << point << std::endl;

			fprintf(fp, "%.1f ", point);
		}
		fprintf(fp, "\n");
		fprintf(fp, "\n");
	}
	fclose(fp);
}


bool PhaserGrayCode::PHinit(int N, int Nphase, int HLratio)
{
	this->N = N;
	this->Nphase = Nphase;
	this->HLratio = HLratio;

	cout << "相移法步数:" << N << std::endl;
	cout << "图片总数量:" << Nphase << std::endl;

	return true;
}




bool PhaserGrayCode::init(int N, int n, int n_wb, float I_thr)
{
    // 为了便于大家理解，这里不做参数检查
    this->N = N;
    this->n = n;
    this->n_wb = n_wb;
    this->I_thr = I_thr;
    cout << "############# 初始化参数 #############" << std::endl;
    cout << "相移法步数:" << N << std::endl;
    cout << "格雷码数量:" << n << std::endl;
    cout << "格雷码黑白:" << n_wb << std::endl;
    cout << "格雷码阈值:" << I_thr << std::endl;


	vs_row = makeGrayCode(n);
	for (int i = 0; i < vs_row.size(); ++i)
	{
		V2K.insert(pair<int, int>(vs_row[i], i));
	}

	vs_rowN1 = makeGrayCode(n+1);
	for (int i = 0; i < vs_rowN1.size(); ++i)
	{
		V2KN1.insert(pair<int, int>(vs_rowN1[i], i));
	}


    return true;
}



bool PhaserGrayCode::initCP(int N, int n, int n_wb, float I_thr, float Hnum, float Vnum)
{
	// 为了便于大家理解，这里不做参数检查
	this->N = N;
	this->n = n;
	this->n_wb = n_wb;
	this->I_thr = I_thr;

	this->Hpixelnum = Hnum;  //水平方向一个周期多少个条纹
    this->Vpixelnum = Vnum;  //竖直方向一个周期多少个条纹


	cout << "############# 初始化参数 #############" << std::endl;
	cout << "相移法步数:" << N << std::endl;
	cout << "格雷码数量:" << n << std::endl;
	cout << "格雷码黑白:" << n_wb << std::endl;
	cout << "格雷码阈值:" << I_thr << std::endl;
	cout << "水平方向周期像素:" << Hnum << std::endl;
	cout << "竖直方向周期像素:" << Vnum << std::endl;

	vs_row = makeGrayCode(n);
	for (int i = 0; i < vs_row.size(); ++i)
	{
		V2K.insert(pair<int, int>(vs_row[i], i));
	}

	vs_rowN1 = makeGrayCode(n + 1);
	for (int i = 0; i < vs_rowN1.size(); ++i)
	{
		V2KN1.insert(pair<int, int>(vs_rowN1[i], i));
	}


	return true;
}




//采用多频法进行相位解包裹
bool PhaserGrayCode::MultiScalePhase(vector<cv::String> &files, cv::Mat &pha, cv::Mat &B)
{

	vector<cv::String> files_phaseShift;
	vector<cv::Mat> imgs_phaseShift;
	vector<cv::Mat> imgs_phase;

	// 01 读取图片
	for (int i = 0; i < Nphase; ++i)
	{
		cv::Mat img = cv::imread(files[i], 0);
		// 高斯滤波
		cv::GaussianBlur(img, img, cv::Size(3, 3), 1, 1);
		// 转化为float类型
		img.convertTo(img, CV_64FC1);
		if (img.empty()) {
			cerr << "读取图片" << files[i] << "失败" << std::endl;
			throw errc::io_error;
		}

		// 相移
		imgs_phaseShift.push_back(img);
		files_phaseShift.push_back(files[i]);
	}

	//多频法各个阶段解调
	calc_Multiscale_phase_shift(imgs_phaseShift, imgs_phase, B, N);

	cv::Mat PhaW = imgs_phase[0] * 35;
	//imwrite("jiPhaW.png", PhaW);

	imwrite("phase1.png", PhaW);
	cv::Mat PhaW1 = imgs_phase[1] * 35;
	imwrite("phase2.png", PhaW1);
	PhaW = imgs_phase[2] * 35;
	imwrite("phase3.png", PhaW);

	//多频法解包裹
	phaseunwrap(imgs_phase, pha);

	
	return true;
}



bool PhaserGrayCode::phaseunwrap(vector<cv::Mat> &wrapedphase, cv::Mat &unwrapedphase)
{
	float k = HLratio;
	float n = 0.0f;
	int MN = wrapedphase.size();

	img_size = wrapedphase[0].size();
	unwrapedphase = cv::Mat::zeros(img_size, CV_64FC1);
	cv::Mat phaseH = cv::Mat::zeros(img_size, CV_64FC1);

	unwrapedphase = wrapedphase[0];

	for (int m = 1; m < MN; m++)
	{
		phaseH = wrapedphase[m];

		for (int row = 0; row < unwrapedphase.rows; row++)
		{
			for (int col = 0; col < unwrapedphase.cols; col++)
			{
				n = round((k*unwrapedphase.at<double>(row, col) - phaseH.at<double>(row, col)) / (2.0f*M_PI));
				unwrapedphase.at<double>(row, col) = phaseH.at<double>(row, col) + 2.0f*n*M_PI;
			}
		}
	}
	return 1;
}


// 相移法，计算包裹相位
void PhaserGrayCode::calc_Multiscale_phase_shift(vector<cv::Mat> &imgs, vector<cv::Mat> &pha, cv::Mat &B, int N)
{
	//B--
	//N--相移步数
	
	int M = Nphase/N;

	img_size = imgs[0].size();
	

	B = cv::Mat::zeros(img_size, CV_64FC1);
	cv::Mat Btemp = cv::Mat::zeros(img_size, CV_64FC1);
	cv::Mat Ik = cv::Mat::zeros(img_size, CV_64FC1);
	cv::Mat sin_sum = cv::Mat::zeros(img_size, CV_64FC1);
	cv::Mat cos_sum = cv::Mat::zeros(img_size, CV_64FC1);


	for (int m = 0; m < M; m++)
	{
		//push_back()传入的是指针，从而用于计算的地址应该每次从新开辟。
		//否则，最后Vector中所有图像是最后一次计算得到的图像
		cv::Mat phaTemp = cv::Mat::zeros(img_size, CV_64FC1);

		sin_sum = 0;
		cos_sum = 0;

		for (int k = 0; k < N; k++)
		{
			Ik = imgs[m*N+k];
			sin_sum = sin_sum + Ik * sin(2. * float(k) * M_PI / double(N));
			cos_sum = cos_sum + Ik * cos(2. * float(k) * M_PI / double(N));
		}

		// 计算相位、调制度
        #pragma omp parallel for
		for (int row = 0; row < Ik.rows; ++row)
		{
			for (int col = 0; col < Ik.cols; ++col)
			{
				double s = sin_sum.at<double>(row, col);
				double c = cos_sum.at<double>(row, col);
				phaTemp.at<double>(row, col) = atan2(s, c);
				Btemp.at<double>(row, col) = sqrt(s * s + c * c) * 2 / double(N);
			}
		}

		if (m == 0)
			B = Btemp;


		phaTemp = -1. * phaTemp;
		cv::Mat pha_low_mask = (phaTemp <= 0.) / 255.;
		pha_low_mask.convertTo(pha_low_mask, CV_64FC1);
		phaTemp = phaTemp + pha_low_mask * 2. * M_PI;


		cv::Mat PhaW = phaTemp * 35;
		imwrite("phasTemp.png", PhaW);

		if (m == 2)
		{
			cv::Mat PhaW1 = pha[0] * 35;
			imwrite("phasTemp1.png", PhaW1);
		}

		pha.push_back(phaTemp);
	}


}



// 相移法，计算包裹相位
bool PhaserGrayCode::calc_phase_shift(vector<cv::Mat> &imgs, cv::Mat &pha, cv::Mat &GrayImg, cv::Mat &B, int N)
{
	//B--
	//N--相移步数
	img_size = imgs[0].size();
	pha = cv::Mat::zeros(img_size, CV_64FC1);
	B = cv::Mat::zeros(img_size, CV_64FC1);
	//GrayImg = cv::Mat::zeros(img_size, CV_64FC1);

	cv::Mat Ik = cv::Mat::zeros(img_size, CV_64FC1);
	cv::Mat sin_sum = cv::Mat::zeros(img_size, CV_64FC1);
	cv::Mat cos_sum = cv::Mat::zeros(img_size, CV_64FC1);

	for (int k = 0; k < N; k++)
	{
		Ik = imgs[k];
		sin_sum = sin_sum + Ik * sin(2. * float(k) * M_PI / double(N));
		cos_sum = cos_sum + Ik * cos(2. * float(k) * M_PI / double(N));

		//相移图的均值，用于阈值处理
		//GrayImg = GrayImg+Ik /double(N);
		
        #pragma omp parallel for
		for (int row = 0; row < Ik.rows; ++row)
		{
			for (int col = 0; col < Ik.cols; ++col)
			{
				GrayImg.at<double>(row, col) += Ik.at<double>(row, col) / double(N)/255.0f;
			}
		}



	}

	// 计算相位、调制度
    #pragma omp parallel for
	for (int row = 0; row < Ik.rows; ++row)
	{
		for (int col = 0; col < Ik.cols; ++col)
		{
			double s = sin_sum.at<double>(row, col);
			double c = cos_sum.at<double>(row, col);
			pha.at<double>(row, col) = atan2(s, c);
			B.at<double>(row, col) = sqrt(s * s + c * c) * 2 / double(N);
		}
	}


	pha = -1. * pha;
	//cv::Mat pha_low_mask = (pha <= 0.) / 255.;
	//pha_low_mask.convertTo(pha_low_mask, CV_64FC1);

	//对小于0的相位加上 2*M_PI,从而转换到一个周期之内
	//pha = pha + pha_low_mask * 2. * M_PI;


     #pragma omp parallel for
	for (int row = 0; row < Ik.rows; ++row)
	{
		for (int col = 0; col < Ik.cols; ++col)
		{
			if (pha.at<double>(row, col) <0.0f)
			  pha.at<double>(row, col) += 2.0*M_PI;

			pha.at<double>(row, col) -= M_PI;
		}
	}


	return !pha.empty();
}




//相机和投影仪标定中的相位计算函数
/*
   相机和投影仪标定；
*/
bool PhaserGrayCode::calcPhaseCPcalibration(vector<cv::String>& files, cv::Mat& phaH, cv::Mat& phaV, cv::Mat &Img)
{

	//读取所有的图像
	int Imgnum = files.size();
	vector<cv::Mat> AImage;
	for (int i = 0; i < Imgnum; ++i)
	{
		cv::Mat img = cv::imread(files[i], 0);
		AImage.push_back(img);
		
	}

	//横竖条纹数量
	int Num = Imgnum / 2;
	vector<cv::Mat> HImage;
	vector<cv::Mat> VImage;

	for (int i = 0; i < Imgnum; ++i)
	{
		if (i < Num)
		{
			HImage.push_back(AImage[i]);
		}
		else
		{
			VImage.push_back(AImage[i]);
		}

	}

	//
	img_size = HImage[0].size();
	//对相位进行求解
	cv::Mat PhaseH = cv::Mat::zeros(img_size, CV_64FC1);
	cv::Mat	PhaseV = cv::Mat::zeros(img_size, CV_64FC1);
	phaH = cv::Mat::zeros(img_size, CV_64FC1);
	phaV = cv::Mat::zeros(img_size, CV_64FC1);
	Img = cv::Mat::zeros(img_size, CV_64FC1);

	//保存一张图片。用于角点检测
	//获取一张图像用于检测角点；主要用于角点提取
	HImage[Num-1].convertTo(Img, CV_64FC1);
	//相位计算，用于计算相位
	PhaseCPcalibration(HImage, PhaseH);
	PhaseCPcalibration(VImage, PhaseV);


	//
	m_h = HImage[0].rows;
	m_w = HImage[0].cols;

	for (int h = 0; h < m_h; h++)
	{
		for (int w = 0; w < m_w; w++)
		{
			
			phaH.at<double>(h, w) = PhaseH.at<double>(h, w) ;
			phaV.at<double>(h, w) = PhaseV.at<double>(h, w);
			
		}
	}


	return true;

}



/*
   相位标定的主程序入口；主要有读入等
*/
bool PhaserGrayCode::PhaseCPcalibration(vector<cv::Mat>& files, cv::Mat& pha)
{

	//vector<cv::String> files_phaseShift, files_grayCode;
	vector<cv::Mat> imgs_phaseShift, imgs_grayCode, imgs_GCCode;
	cv::Mat GrayImg;



	// 01 读取图片
	for (int i = 0; i < N + n + 3; ++i)
	{
		cv::Mat img = files[i];
		// 高斯滤波
		cv::GaussianBlur(img, img, cv::Size(3, 3), 1, 1);
		// 转化为float类型
		img.convertTo(img, CV_64FC1);
		
		// 相移
		if (i < N)
		{
			imgs_phaseShift.push_back(img);
			//files_phaseShift.push_back(files[i]);
		}
		// 格雷码
		else
		{
			//读入了所有5张格雷码和 两张黑白
			imgs_grayCode.push_back(img);
			//files_grayCode.push_back(files[i]);
		}
	}



	img_size = imgs_grayCode[0].size();
	pha = cv::Mat::zeros(img_size, CV_64FC1);
	cv::Mat B = cv::Mat::zeros(img_size, CV_64FC1);
	GrayImg = cv::Mat::zeros(img_size, CV_64FC1);

	m_h = imgs_grayCode[0].rows;
	m_w = imgs_grayCode[0].cols;

	// 02 解码相移条纹，phs_wrapped为输出的含包裹相位
	//一个周期内的相位为 [-Pi,Pi]
	calc_phase_shift(imgs_phaseShift, pha_wrapped, GrayImg, B, N);

	//cv::Mat BL = (B > 15);
	saveTXTFF("PhaseTestImg.txt", pha_wrapped);

	cv::Mat B_mask = (B > 15) / 255.;
	B_mask.convertTo(B_mask, CV_64FC1);
	cv::Mat phatemp = pha_wrapped.mul(B_mask);
	//cv::Mat phatemp = pha_wrapped * BL;


	cv::Mat PhaW = (phatemp + 2.0 * M_PI) * 25;
	imwrite("jiPhaW.png", PhaW);

	cv::Mat PhaW2 = (pha_wrapped + 2.0 * M_PI) * 25;
	imwrite("jiPhaW2.png", PhaW2);
	// 03 解码格雷码图案，n为格雷码图片
	// 此处实现的是两类格雷码的计算，互补格雷码
	// 一个是n 一个是n+1; 不在需要阈值了

	//添加一个采用相移图像值进行二值化的程序
	//Mycalc_gcs(imgs_grayCode, GrayImg, imgs_GCCode);

	calc_gcs(imgs_grayCode, I_thr, imgs_GCCode);

	//K1解出来的条纹级数
	calc_gray_code(imgs_GCCode, ks, I_thr, n);

	//K2解出来的条纹级数
	calc_gray_codeN1(imgs_GCCode, ks2, I_thr, n + 1);

	//saveTXT("K2T.txt", ks2);

#pragma omp parallel for
	for (int h = 0; h < m_h; h++)
	{
		for (int w = 0; w < m_w; w++)
		{
			ks2.at<double>(h, w) = int((ks2.at<double>(h, w) + 1) / 2);
		}
	}


	cv::Mat KS = ks * 17;
	cv::Mat KS2 = ks2 * 17;

	imwrite("jishu.png", KS);
	imwrite("jishu2.png", KS2);

	//saveTXT("K1.txt", ks);
	//saveTXT("K2.txt", ks2);

	// 04 相位展开

	//pha = pha_wrapped + 2. * M_PI * ks;



#pragma omp parallel for
	for (int h = 0; h < m_h; h++)
	{
		for (int w = 0; w < m_w; w++)
		{
			if (pha_wrapped.at<double>(h, w) <= -M_PI / 2.0f)
			{
				pha.at<double>(h, w) = (pha_wrapped.at<double>(h, w) + M_PI) + 2. * M_PI * ks2.at<double>(h, w);
			}
			else if (pha_wrapped.at<double>(h, w) >= M_PI / 2.0f)
			{
				pha.at<double>(h, w) = (pha_wrapped.at<double>(h, w) + M_PI) + 2. * M_PI * ks2.at<double>(h, w) - 2. * M_PI;
			}
			else
			{
				pha.at<double>(h, w) = (pha_wrapped.at<double>(h, w) + M_PI) + 2. * M_PI * ks.at<double>(h, w);
			}
		}
	}


}




//计算相位信息
//此处修改成: 互补格雷码方法+相移法
// 采用多步相移法的均值来作为像素点级别的阈值

bool PhaserGrayCode::calcPhase(vector<cv::String> &files, cv::Mat &pha, cv::Mat &B)
{

    //vector<cv::String> files_phaseShift, files_grayCode;
    vector<cv::Mat> imgs_phaseShift, imgs_grayCode, imgs_GCCode;
	cv::Mat GrayImg;
    // 01 读取图片
    for (int i = 0; i < N + n +3; ++i)
	{
        cv::Mat img = cv::imread(files[i], 0);
        // 高斯滤波
        cv::GaussianBlur(img, img, cv::Size(3, 3), 1, 1);
        // 转化为float类型
        img.convertTo(img, CV_64FC1);
        if (img.empty()){
            cerr << "读取图片" << files[i] << "失败" << std::endl;
            throw errc::io_error;
        }

        // 相移
        if (i < N)
		{
            imgs_phaseShift.push_back(img);
            //files_phaseShift.push_back(files[i]);
        }
        // 格雷码
        else
		{
			//读入了所有5张格雷码和 两张黑白
            imgs_grayCode.push_back(img);
            //files_grayCode.push_back(files[i]);
        }
    }


	
	img_size = imgs_grayCode[0].size();
	pha = cv::Mat::zeros(img_size, CV_64FC1);
	GrayImg= cv::Mat::zeros(img_size, CV_64FC1);

	m_h = imgs_grayCode[0].rows;
	m_w = imgs_grayCode[0].cols;

    // 02 解码相移条纹，phs_wrapped为输出的含包裹相位
	//一个周期内的相位为 [-Pi,Pi]
    calc_phase_shift(imgs_phaseShift, pha_wrapped, GrayImg, B, N);

	//cv::Mat BL = (B > 15);
	saveTXTFF("PhaseTestImg.txt", pha_wrapped);

	cv::Mat B_mask = (B > 15) / 255.;
	B_mask.convertTo(B_mask, CV_64FC1);
	cv::Mat phatemp = pha_wrapped.mul(B_mask);
	//cv::Mat phatemp = pha_wrapped * BL;


	cv::Mat PhaW = (phatemp+2.0*M_PI) * 25;
    imwrite("jiPhaW.png", PhaW);

	cv::Mat PhaW2 = (pha_wrapped + 2.0 * M_PI)*25;
	imwrite("jiPhaW2.png", PhaW2);
    // 03 解码格雷码图案，n为格雷码图片
	// 此处实现的是两类格雷码的计算，互补格雷码
	// 一个是n 一个是n+1; 不在需要阈值了

	//添加一个采用相移图像值进行二值化的程序
	//Mycalc_gcs(imgs_grayCode, GrayImg, imgs_GCCode);

	calc_gcs(imgs_grayCode, I_thr, imgs_GCCode);
    
	//K1解出来的条纹级数
	calc_gray_code(imgs_GCCode, ks, I_thr, n);

	//K2解出来的条纹级数
	calc_gray_codeN1(imgs_GCCode, ks2, I_thr, n+1);

	//saveTXT("K2T.txt", ks2);

    #pragma omp parallel for
	for (int h = 0; h < m_h; h++)
	{
		for (int w = 0; w < m_w; w++)
		{
			ks2.at<double>(h, w) = int((ks2.at<double>(h, w) + 1) / 2);
		}
	}


	cv::Mat KS = ks * 17;
	cv::Mat KS2 = ks2 * 17;

	imwrite("jishu.png", KS);
	imwrite("jishu2.png", KS2);

	//saveTXT("K1.txt", ks);
	//saveTXT("K2.txt", ks2);

    // 04 相位展开
    
	//pha = pha_wrapped + 2. * M_PI * ks;


	
    #pragma omp parallel for
	for (int h = 0; h < m_h; h++)
	{
		for (int w = 0; w < m_w; w++)
		{
			if (pha_wrapped.at<double>(h, w) <= -M_PI / 2.0f)
			{
				pha.at<double>(h,w) = (pha_wrapped.at<double>(h,w)+M_PI) + 2. * M_PI * ks2.at<double>(h,w);
			}
			else if(pha_wrapped.at<double>(h, w) >= M_PI / 2.0f)
			{
				pha.at<double>(h, w) = (pha_wrapped.at<double>(h, w)+M_PI) + 2. * M_PI * ks2.at<double>(h, w)-2.*M_PI;
			}
			else
			{
				pha.at<double>(h, w) = (pha_wrapped.at<double>(h, w)+M_PI) + 2. * M_PI * ks.at<double>(h, w);
			}
		}
	}
	





    return true;

}


//计算相位信息
//此处修改成: 互补格雷码方法+相移法
// 采用多步相移法的均值来作为像素点级别的阈值
bool PhaserGrayCode::CPcalcPhase(vector<cv::String>& files, cv::Mat& phaH, cv::Mat& phaW)
{

	//vector<cv::String> files_phaseShift, files_grayCode;
	vector<cv::Mat> imgs_phaseShiftH, imgs_grayCodeH, imgs_GCCodeH;
	vector<cv::Mat> imgs_phaseShiftW, imgs_grayCodeW, imgs_GCCodeW;
	

	cv::Mat GrayImg;
	// 01 读取图片
	for (int i = 0; i < 20; ++i)
	{
		cv::Mat img = cv::imread(files[i], 0);
		// 高斯滤波
		cv::GaussianBlur(img, img, cv::Size(3, 3), 1, 1);
		// 转化为float类型
		img.convertTo(img, CV_64FC1);
		if (img.empty()) {
			cerr << "读取图片" << files[i] << "失败" << std::endl;
			throw errc::io_error;
		}

		// 相移
		if (i < 3)
		{
			imgs_phaseShiftH.push_back(img);
		}
		// 格雷码
		else if(i>2&&i<10)
		{
			//读入了所有5张格雷码和 两张黑白
			imgs_grayCodeH.push_back(img);
			
		}
		else if (i > 9 && i < 13)
		{
			imgs_phaseShiftW.push_back(img);
		}
		else
		{
			imgs_grayCodeW.push_back(img);
		}


	}



	img_size = imgs_grayCodeH[0].size();
	phaH = cv::Mat::zeros(img_size, CV_64FC1);
	phaW = cv::Mat::zeros(img_size, CV_64FC1);

	GrayImg = cv::Mat::zeros(img_size, CV_64FC1);

	m_h = imgs_grayCodeH[0].rows;
	m_w = imgs_grayCodeH[0].cols;

	//*******************第一个方向计算*******************************
	cv::Mat B1;
	calc_phase_shift(imgs_phaseShiftH, pha_wrapped, GrayImg, B1, N);
	calc_gcs(imgs_grayCodeH, I_thr, imgs_GCCodeH);

	//K1解出来的条纹级数
	calc_gray_code(imgs_GCCodeH, ks, I_thr, n);

	//K2解出来的条纹级数
	calc_gray_codeN1(imgs_GCCodeH, ks2, I_thr, n + 1);

	//saveTXT("K2T.txt", ks2);
    #pragma omp parallel for
	for (int h = 0; h < m_h; h++)
	{
		for (int w = 0; w < m_w; w++)
		{
			ks2.at<double>(h, w) = int((ks2.at<double>(h, w) + 1) / 2);
		}
	}
    #pragma omp parallel for
	for (int h = 0; h < m_h; h++)
	{
		for (int w = 0; w < m_w; w++)
		{
			if (pha_wrapped.at<double>(h, w) <= -M_PI / 2.0f)
			{
				phaH.at<double>(h, w) = (pha_wrapped.at<double>(h, w) + M_PI) + 2. * M_PI * ks2.at<double>(h, w);
			}
			else if (pha_wrapped.at<double>(h, w) >= M_PI / 2.0f)
			{
				phaH.at<double>(h, w) = (pha_wrapped.at<double>(h, w) + M_PI) + 2. * M_PI * ks2.at<double>(h, w) - 2. * M_PI;
			}
			else
			{
				phaH.at<double>(h, w) = (pha_wrapped.at<double>(h, w) + M_PI) + 2. * M_PI * ks.at<double>(h, w);
			}
		}
	}


	//*******************第二个方向计算*******************************
	cv::Mat B2;
	calc_phase_shift(imgs_phaseShiftH, pha_wrapped, GrayImg, B2, N);
	calc_gcs(imgs_grayCodeW, I_thr, imgs_GCCodeW);

	//K1解出来的条纹级数
	calc_gray_code(imgs_GCCodeW, ks, I_thr, n);

	//K2解出来的条纹级数
	calc_gray_codeN1(imgs_GCCodeW, ks2, I_thr, n + 1);

	//saveTXT("K2T.txt", ks2);
    #pragma omp parallel for
	for (int h = 0; h < m_h; h++)
	{
		for (int w = 0; w < m_w; w++)
		{
			ks2.at<double>(h, w) = int((ks2.at<double>(h, w) + 1) / 2);
		}
	}
    #pragma omp parallel for
	for (int h = 0; h < m_h; h++)
	{
		for (int w = 0; w < m_w; w++)
		{
			if (pha_wrapped.at<double>(h, w) <= -M_PI / 2.0f)
			{
				phaW.at<double>(h, w) = (pha_wrapped.at<double>(h, w) + M_PI) + 2. * M_PI * ks2.at<double>(h, w);
			}
			else if (pha_wrapped.at<double>(h, w) >= M_PI / 2.0f)
			{
				phaW.at<double>(h, w) = (pha_wrapped.at<double>(h, w) + M_PI) + 2. * M_PI * ks2.at<double>(h, w) - 2. * M_PI;
			}
			else
			{
				phaW.at<double>(h, w) = (pha_wrapped.at<double>(h, w) + M_PI) + 2. * M_PI * ks.at<double>(h, w);
			}
		}
	}



	return true;

}






//
bool PhaserGrayCode::calc_gray_codeN1(vector<cv::Mat> &gcs, cv::Mat &ks, double I_thr, int n)
{
	
	
	map<int, int>::iterator it;
	
	// 04 一一映射，将V转化为K
	ks = cv::Mat::zeros(img_size, CV_64FC1);

	#pragma omp parallel for
	//此处用并行计算会出错
	for (int row = 0; row < gcs[0].rows; ++row)
	{
		for (int col = 0; col < gcs[0].cols; ++col)
		{
			double v = 0;
			for (int i = 0; i < n; ++i)
			{
				v = v + gcs[i].at<double>(row, col) * pow(2, n - (i + 1));
			}
			
			ks.at<double>(row, col) = v;
			
		}
	}

    //#pragma omp parallel for
	for (int row = 0; row < gcs[0].rows; ++row)
	{
		for (int col = 0; col < gcs[0].cols; ++col)
		{
			// 查找元素
			double v = 0.;
			v = ks.at<double>(row, col);
			it = V2KN1.find(int(v));
			if (it != V2KN1.end())
			{
				int k = it->second;
				ks.at<double>(row, col) = double(k);
			}
			else
			{
				cerr << "解码错误" << std::endl;
				throw - 1;
			}
		}
	}



	return true;
}


//
bool PhaserGrayCode::calc_gray_code(vector<cv::Mat> &gcs, cv::Mat &ks, double I_thr, int n) 
{
    
	map<int, int>::iterator it;
    // 04 一一映射，将V转化为K
    ks = cv::Mat::zeros(img_size, CV_64FC1);

    #pragma omp parallel for
	//此处用并行计算会出错
    for (int row = 0; row < gcs[0].rows; ++row)
	{
        for (int col = 0; col < gcs[0].cols; ++col)
		{
            double v = 0;
            for (int i = 0; i < n; ++i) 
			{
                v = v + gcs[i].at<double>(row, col) * pow(2, n - (i + 1));
            }

			ks.at<double>(row, col) = v;

            
        }
    }


    //#pragma omp parallel for
	for (int row = 0; row < gcs[0].rows; ++row)
	{
		
		for (int col = 0; col < gcs[0].cols; ++col)
		{
			// 查找元素
			double v = 0.;
			v=ks.at<double>(row, col);
			it = V2K.find(int(v));
			if (it != V2K.end())
			{
				int k = it->second;
				ks.at<double>(row, col) = double(k);
			}
			else
			{
				cerr << "解码错误" << std::endl;
				throw - 1;
			}
		}
	}

    return true;
}





bool PhaserGrayCode::Mycalc_gcs(vector<cv::Mat> &imgs, cv::Mat &GrayImg, vector<cv::Mat> &gcs)
{
	// 01 计算最大、最小值
	cv::Mat I_max = cv::Mat::zeros(img_size, CV_64FC1);
	
	double I = 0.0f, thres = 0.0f;;

	for (auto &img : imgs)
	{
        //#pragma omp parallel for
		for (int row = 0; row < img.rows; ++row)
		{
			for (int col = 0; col < img.cols; ++col)
			{

				I = img.at<double>(row, col)/255.0f;
				thres = GrayImg.at<double>(row, col);

				if (I > thres)
					I_max.at<double>(row, col) = 255.;
				else
					I_max.at<double>(row, col) = 0.;


			}
		}

		imwrite("GrayImgT.png", I_max);
		gcs.push_back(I_max/255.);
	}

	
	return !gcs.empty();
}




bool PhaserGrayCode::calc_gcs(vector<cv::Mat> &imgs, double I_thr, vector<cv::Mat> &gcs) 
{
    // 01 计算最大、最小值
    cv::Mat I_max = cv::Mat::zeros(img_size, CV_64FC1);
    cv::Mat I_min = cv::Mat::zeros(img_size, CV_64FC1);
    for (auto &img: imgs)
	{
         #pragma omp parallel for
        for (int row = 0; row < img.rows; ++row) 
		{
            for (int col = 0; col < img.cols; ++col)
			{

                double I = img.at<double>(row, col);

                if (I > I_max.at<double>(row, col))
				{
                    I_max.at<double>(row, col) = I;
                }
                if (I < I_min.at<double>(row, col))
				{
                    I_min.at<double>(row, col) = I;
                }
            }
        }
    }

    // 02 归一化，并转化为GC码
	//进行归一化处理，归一化之后像素值的范围为[0,1]
	//这种方法需要额外的投射全黑或者全白的图片
    for (auto &img: imgs)
	{
        cv::Mat gc = ((img - I_min) / (I_max - I_min) > I_thr);
        gc.convertTo(gc, CV_64FC1);
        gcs.push_back(gc / 255.);
    }


    return !gcs.empty();
}



//这儿就是计算V值的
vector<int> PhaserGrayCode::makeGrayCode(int n) 
{
    vector<int> res;
    // 特殊情况
    res.push_back(0);
    if (n == 0)
        return res;
    res.push_back(1);
    if (n == 1)
        return res;
    int add = 1;

    for (int i = 1; i < n; i++)
	{
        add *= 2;
        for (int j = res.size() - 1; j >= 0; j--)
		{
            res.push_back(res[j] + add);
        }
    }
    return res;
}


// 梯度滤波：去除异常点
bool PhaserGrayCode::filterGradient(cv::Mat &img_src, cv::Mat& img_des) 
{
    // 01 梯度滤波
    cv::Mat dx, dy;
    cv::Sobel(img_src, dx, img_src.depth(), 1, 0, 1, 0.5, 0); // x方向
    cv::Sobel(img_src, dy, img_src.depth(), 0, 1, 1, 0.5, 0); // y方向

    float d_thr = 1;
    cv::Mat dx_mask = (cv::abs(dx) < d_thr) / 255.;
    cv::Mat dy_mask = (cv::abs(dy) < d_thr) / 255.;

    dx_mask.convertTo(dx_mask, CV_64FC1);
    dy_mask.convertTo(dy_mask, CV_64FC1);

    cv::Mat dxy_mask = dx_mask.mul(dy_mask);
    // 对掩模进行腐蚀（只要有黑的，就将整个变为黑色的）
    // cv::erode(dxy_mask, dxy_mask, cv::Mat());

    // 梯度异常的区域，我们将其设置为边上区域的中值
    img_des = img_src.clone();

    #pragma omp parallel for
    for (int row = 0; row < dxy_mask.rows; ++row)
	{
        for (int col = 0; col < dxy_mask.cols; ++col) 
		{
            // 如果异常，填入中值（框必须足够大，否则都是异常值）
            if (dxy_mask.at<double>(row, col) < 0.5)
			{
                img_des.at<double>(row, col) = 0;
            }
        }
    }
    return true;
}

bool PhaserGrayCode::filterB(cv::Mat &img_src, cv::Mat &img_des, cv::Mat &B, float B_min)
{
    cv::Mat B_mask = (B > B_min) / 255.;
    B_mask.convertTo(B_mask, CV_64FC1);
    img_des = img_src.mul(B_mask);
    return true;
}









