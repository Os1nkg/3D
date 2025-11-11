#include "PointJoint.h"
//#include "Spline.h"



bool PointJoint::init(float pha_dif)
{
    
    this->dis_dif = pha_dif;
    return false;
}

float PointJoint::Ediatance(cv::Point3f& A, cv::Point3f& B)
{
	float temp = 0;
	temp = (A.x - B.x)*(A.x - B.x);
	temp += (A.y - B.y) * (A.y - B.y);
	temp += (A.z - B.z) * (A.z - B.z);
	temp = sqrtf(temp);
	return temp;
}


float** GetArray(int row, int col)//动态开辟一个二维数组
{
	if (row < 1 || col < 1)
	{
		return NULL;
	}
	float ** s = (float **)malloc(sizeof(float*) * row);
	if (NULL == s)return NULL;
	for (int i = 0; i < row; ++i)
	{
		s[i] = (float*)malloc(sizeof(float) * col);
		if (s[i] == NULL)//如果在申请空间过程中实现异常则释放
		{
			for (int j = i - 1; j >= 0; --j)
			{
				free(s[j]);
			}
			free(s);
			return NULL;
		}
	}
	return s;
}


void Destory(float ** s, int row)//销毁函数
{
	if (s == NULL)
	{
		return;
	}
	for (int i = 0; i < row; ++i)
	{
		free(s[i]);
	}
	free(s);
	s = NULL;
}


void PointJoint::Initialdata(float **Point,vector<cv::Point3f>& Left)
{
	//
	int Lsize = Left.size();
	for (int y = 0; y < Lsize-1; y++)
	{
		cv::Point3f Start = Left[y];
		for (int x = 0,xt=0; x < Lsize-1; x++,xt++)
		{
			if (xt == y)
				xt = xt + 1;

			cv::Point3f End = Left[xt];
			Point[y][x] = Ediatance(Start, End);
		}
	}

	//最后一行
	
	cv::Point3f Start = Left[Lsize-1];
	for (int x = 0; x < Lsize - 1; x++)
	{
		cv::Point3f End = Left[x];
		Point[Lsize-1][x] = Ediatance(Start, End);
	}
	

}


void PointJoint::Point3DMatch(vector<cv::Point3f>& Left, vector<cv::Point3f>& Right, vector<cv::Point3f>& Leftout, vector<cv::Point3f>& Rightout)
{
	//对于3D点的匹配中，首先需要
	PointMatch(Left, Right, Leftout, Rightout);

	int sizenum = Leftout.size();
	int calnum = 0;

	/*
	while (sizenum >= 9&&calnum<9)
	{
		Left.clear();
		Right.clear();

		int Lnum = Leftout.size();
		int Rnum = Rightout.size();

		for(int k=0;k<Lnum;k++)
		Left.push_back(Leftout[k]);

		for(int m=0;m<Rnum;m++)
		Right.push_back(Rightout[m]);

		PointMatch(Left, Right, Leftout, Rightout);   //
		sizenum = Leftout.size();
		calnum++;

		cout << "--优化迭代一次--"  << sizenum<<std::endl;
	}
	*/

	cout << "--优化迭代完成--" << std::endl;
	//如果一次优化后点数小于3，则采用上一次的结果
	if (sizenum < 3)
	{
		Leftout.clear();
		Rightout.clear();

		int Lnum = Left.size();
		int Rnum = Right.size();

		for (int k = 0; k < Lnum; k++)
			Leftout.push_back(Left[k]);

		for (int m = 0; m < Rnum; m++)
			Rightout.push_back(Right[m]);

	
	}
    

	



}


void PointJoint::FindXYP(vector<cv::Point3f>& Input1, vector<cv::Point3f>& Input2, vector<int>& Lable)
{
	int Anum = Input1.size();
	int Bnum = Input2.size();
	//
	for (int k = 0; k < Bnum; k++)
	{
		for (int l = 0; l < Anum; l++)
		{
			cv::Point3f Temp = Input1[l];
			cv::Point3f Temp1 = Input2[k];
			float distance = Ediatance(Temp, Temp1);
			if (distance < 5)
			{
				Lable.push_back(l);
			}

		}
	}

}




void PointJoint::PointMatch3(vector<cv::Point3f>& Left, vector<cv::Point3f>& Right, vector<cv::Point3f>& Leftout, vector<cv::Point3f>& Rightout)
{
	int Lnum = Left.size();
	int Rnum = Right.size();

	if (Lnum < 5 || Rnum < 5)
	{
		cout << "标志点数量太少！ 请更换方法" << std::endl;
		cout << "第一视角标志点个数: " << Lnum<<std::endl;
		cout << "第二视角标志点个数: " << Rnum << std::endl;
		return;
	}


	float** PointJL, ** PointJR;
	//计算每个点最近的三个点
	vector<vector<cv::Point3f>> LeftA(Lnum);
	vector<vector<cv::Point3f>> RightA(Rnum);
	//找到每个点的最近的三个点
	MinDistance3(Left, LeftA);
	MinDistance3(Right, RightA);
	//
	PointJL = GetArray(Lnum, 5);
	PointJR = GetArray(Rnum, 5);
	//得到三角形的三个边长
	for (int k = 0; k < Lnum; k++)
	{
		//当前点与三个邻域点距离关系
		PointJL[k][0] = Ediatance(LeftA[k][0], LeftA[k][1]);
		PointJL[k][1] = Ediatance(LeftA[k][0], LeftA[k][2]);
		PointJL[k][2] = Ediatance(LeftA[k][0], LeftA[k][3]);

		//三个邻域点顺序关系
		PointJL[k][3] = Ediatance(LeftA[k][1], LeftA[k][2]);
		PointJL[k][4] = Ediatance(LeftA[k][2], LeftA[k][3]);

	}

	for (int k = 0; k < Rnum; k++)
	{
		//当前点与三个邻域点距离关系
		PointJR[k][0] = Ediatance(RightA[k][0], RightA[k][1]);
		PointJR[k][1] = Ediatance(RightA[k][0], RightA[k][2]);
		PointJR[k][2] = Ediatance(RightA[k][0], RightA[k][3]);

		//三个邻域点顺序关系
		PointJR[k][3] = Ediatance(RightA[k][1], RightA[k][2]);
		PointJR[k][4] = Ediatance(RightA[k][2], RightA[k][3]);
		
	}
	//找到三个边长近似一致的一个三角形
	int km = 0, kn = 0;
	int num = 0;
	float threshold = 0.1;
	float a = 0.0f, b = 0.0f, c = 0.0f, d=0.0f, e=0.0f;
	while (!num)
	{
		for (int m = 0; m < Lnum; m++)
		{
			for (int n = 0; n < Rnum; n++)
			{
				a = abs(PointJL[m][0] - PointJR[n][0]);
				b = abs(PointJL[m][1] - PointJR[n][1]);
				c = abs(PointJL[m][2] - PointJR[n][2]);
				d = abs(PointJL[m][3] - PointJR[n][3]);
				e = abs(PointJL[m][4] - PointJR[n][4]);
				//
				if (a < threshold && b < threshold && c < threshold&&d<threshold&& e < threshold)
				{
					km = m;
					kn = n;
					num++;
				}


			}
		}
		threshold += dis_dif * 0.05;
	}


	for (int k = 0; k < 4; k++)
	{
		Leftout.push_back(LeftA[km][k]);
		Rightout.push_back(RightA[kn][k]);
	}



	//动态销毁内存
	Destory(PointJL, 5);
	Destory(PointJR, 5);
}


void PointJoint::MinDistance3(vector<cv::Point3f>& Left, vector<vector<cv::Point3f>>& LeftA)
{
	int Lnum = Left.size();
	double maxdistance = 1000000;
	int knum = 0;
	int knum1 = 0,knum2=0;
	for (int k = 0; k < Lnum; k++)
	{
		cv::Point3f Temp = Left[k];
		LeftA[k].push_back(Temp);  //第一个点

		maxdistance = 1000000000.0f;
		for (int l = 0; l < Lnum; l++)
		{
			if (l != k)
			{
				cv::Point3f Temp2 = Left[l];
				double distance = Ediatance(Temp, Temp2);
				if (distance < maxdistance)
				{
					maxdistance = distance;
					knum = l;
				}
			}
		}
		LeftA[k].push_back(Left[knum]);


		maxdistance = 1000000000.0f;
		for (int l = 0; l < Lnum; l++)
		{
			if (l != k && l != knum)
			{
				cv::Point3f Temp2 = Left[l];
				double distance = Ediatance(Temp, Temp2);
				if (distance < maxdistance)
				{
					maxdistance = distance;
					knum1 = l;
				}
			}
		}
		LeftA[k].push_back(Left[knum1]);

		maxdistance = 1000000000.0f;
		for (int l = 0; l < Lnum; l++)
		{
			if (l != k && l != knum&&l!=knum1)
			{
				cv::Point3f Temp2 = Left[l];
				double distance = Ediatance(Temp, Temp2);
				if (distance < maxdistance)
				{
					maxdistance = distance;
					knum2 = l;
				}
			}
		}
		LeftA[k].push_back(Left[knum2]);

	}
}




void PointJoint::PointMatch2(vector<cv::Point3f>& Left, vector<cv::Point3f>& Right, vector<cv::Point3f>& Leftout, vector<cv::Point3f>& Rightout)
{
	int Lnum = Left.size();
	int Rnum = Right.size();

	if (Lnum < 4 || Rnum < 4)
	{
		cout << "标志点数量太少！ 方法失效" << std::endl;
		return;
	}

	float** PointJL, ** PointJR;
	//计算每个点最近的二个点
	vector<vector<cv::Point3f>> LeftA(Lnum);
	vector<vector<cv::Point3f>> RightA(Rnum);
	//找到每个点的最近的二个点
	MinDistance2(Left, LeftA);
	MinDistance2(Right, RightA);
	//
	PointJL = GetArray(Lnum, 3);
	PointJR = GetArray(Rnum, 3);
	//得到三角形的三个边长
	for (int k = 0; k < Lnum; k++)
	{
		PointJL[k][0]= Ediatance(LeftA[k][0], LeftA[k][1]);
		PointJL[k][1] = Ediatance(LeftA[k][0], LeftA[k][2]);
		PointJL[k][2] = Ediatance(LeftA[k][1], LeftA[k][2]);
	}

	for (int k = 0; k < Rnum; k++)
	{
		PointJR[k][0] = Ediatance(RightA[k][0], RightA[k][1]);
		PointJR[k][1] = Ediatance(RightA[k][0], RightA[k][2]);
		PointJR[k][2] = Ediatance(RightA[k][1], RightA[k][2]);
	}
	//找到三个边长近似一致的一个三角形
	int km = 0, kn = 0;
	int num = 0;
	float threshold = 0.1;
	float a = 0.0f, b = 0.0f, c = 0.0f;
	while (!num)
	{
		for (int m = 0; m < Lnum; m++)
		{
			for (int n = 0; n < Rnum; n++)
			{
				 a = abs(PointJL[m][0] - PointJR[n][0]);
				 b = abs(PointJL[m][1] - PointJR[n][1]);
				 c = abs(PointJL[m][2] - PointJR[n][2]);

				//
				if (a < threshold && b < threshold && c < threshold)
				{
					km = m;
					kn = n;
					num++;
				}


			}
		}
		threshold += dis_dif *0.1;
	}


	for (int k = 0; k < 3; k++)
	{
		Leftout.push_back(LeftA[km][k]);
		Rightout.push_back(RightA[kn][k]);
	}



	//动态销毁内存
	Destory(PointJL, 3);
	Destory(PointJR, 3);
}



void PointJoint::MinDistance2(vector<cv::Point3f>& Left, vector<vector<cv::Point3f>> &LeftA)
{
	int Lnum = Left.size();
	double maxdistance = 1000000;
	int knum = 0;
	int knum1 = 0;
	for (int k = 0; k < Lnum; k++)
	{
		cv::Point3f Temp = Left[k];
		LeftA[k].push_back(Temp);  //第一个点

		maxdistance = 1000000000.0f;
		for (int l = 0; l < Lnum; l++)
		{
			if (l != k)
			{
				cv::Point3f Temp2 = Left[l];
				double distance = Ediatance(Temp, Temp2);
				if (distance < maxdistance)
				{
					maxdistance = distance;
					knum = l;
				}
			}
		}
		LeftA[k].push_back(Left[knum]);


		maxdistance = 1000000000.0f;
		for (int l = 0; l < Lnum; l++)
		{
			if (l != k && l != knum)
			{
				cv::Point3f Temp2 = Left[l];
				double distance = Ediatance(Temp, Temp2);
				if (distance < maxdistance)
				{
					maxdistance = distance;
					knum1 = l;
				}
			}
		}

		LeftA[k].push_back(Left[knum1]);
	}
}




void PointJoint::PointMatch(vector<cv::Point3f> &Left, vector<cv::Point3f> &Right, vector<cv::Point3f>& Leftout, vector<cv::Point3f>& Rightout)
{
	//点的大小
	int Lsize = Left.size();
	int Rsize = Right.size();

	//
	float** PointJL, ** PointJR;
	//动态开辟内存 [L-1,L-1]
	PointJL = GetArray(Lsize, Lsize-1);
	PointJR = GetArray(Rsize, Rsize-1);
	
	//
	Initialdata(PointJL, Left);
	Initialdata(PointJR, Right);

	vector<int> A, B;

	//开始计算匹配的对应关系
	float disAB = 0;
	int num = 0;
	int bestyr = 0;
	int bestyl = 0;
	int maxnum = 0;
	for (int yl = 0; yl < Lsize; yl++)
	{
		//对Left中一行，即一个点
		
		bestyr = 0;
		bestyl = 0;
		maxnum = 0;
		for (int yr = 0; yr < Rsize; yr++)
		{
			num = 0;
			for (int xl = 0; xl < Lsize-1; xl++)
			{
				for (int xr = 0; xr < Rsize-1; xr++)
				{
					disAB = abs(PointJL[yl][xl] - PointJR[yr][xr]);
					if (disAB < dis_dif)  //估计是匹配点
					{
						num++;
					}
				}
			}

			if (num >= 2) //则可能为对应的匹配点
			{
				if (maxnum < num)
				{
					maxnum = num;
					bestyr = yr;
					//bestyl = yl;
				}
			}
			
		}


		if (num>=2)
		{
			A.push_back(yl);
			B.push_back(bestyr);
		}

	}

	int Lsize2 = A.size();
	int Rsize2 = B.size();

	Leftout.clear();
	Rightout.clear();

	for (int i = 0; i < Lsize2; i++)
	{
		Leftout.push_back(Left[A[i]]);
	}

	for (int i = 0; i < Rsize2; i++)
	{
		Rightout.push_back(Right[B[i]]);
	}

	//动态销毁内存
	Destory(PointJL,Lsize);
	Destory(PointJR, Rsize);

}


void  PointJoint::PointFuse(vector<Point3f>& Left, vector<Point3f>& Right, Mat& R, Mat& t)
{
	// R---3X3矩阵  T---3X1
	int Rpoint_size = Right.size();
	int Lpoint_size = Left.size();
	
	for (int rx = 0; rx < Rpoint_size; rx++)
	{
		Point3f temp = Right[rx];
		//计算平移和旋转后的坐标
		//点云数据进行变换
		temp.x = R.at<double>(0, 0) * temp.x + R.at<double>(0, 1) * temp.y +
			R.at<double>(0, 2) * temp.z + t.at<double>(0, 0);

		temp.y = R.at<double>(1, 0) * temp.x + R.at<double>(1, 1) * temp.y +
			R.at<double>(1, 2) * temp.z + t.at<double>(1, 0);

		temp.z = R.at<double>(2, 0) * temp.x + R.at<double>(2, 1) * temp.y +
			R.at<double>(2, 2) * temp.z + t.at<double>(2, 0);
		
		//点云数据进行融合
		for (int lx = 0; lx < Lpoint_size; lx++)
		{
			Point3f temp1 = Left[lx];
			float distance = Ediatance(temp, temp1);
			if (distance > dis_dif)   //考虑下阈值
				Left.push_back(temp);
		}
	}

}



void  PointJoint::pose_estimation_3d3d(const vector<Point3f>& pts1, const vector<Point3f>& pts2, Mat& R, Mat& t )

{

	Point3f p1, p2;     // center of mass
	int N = pts1.size();

	for (int i = 0; i < N; i++)
	{
		p1 += pts1[i];
		p2 += pts2[i];
	}

	p1 = Point3f(Vec3f(p1) / N);
	p2 = Point3f(Vec3f(p2) / N);
	vector<Point3f>     q1(N), q2(N); // remove the center

	for (int i = 0; i < N; i++)
	{
		q1[i] = pts1[i] - p1;
		q2[i] = pts2[i] - p2;
	}

	// compute q1*q2^T
	Eigen::Matrix3d W = Eigen::Matrix3d::Zero();

	for (int i = 0; i < N; i++)
	{
		W += Eigen::Vector3d(q1[i].x, q1[i].y, q1[i].z) * Eigen::Vector3d(q2[i].x, q2[i].y, q2[i].z).transpose();
	}

	cout << "W=" << W << std::endl;

	// SVD on W

	Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
	Eigen::Matrix3d U = svd.matrixU();
	Eigen::Matrix3d V = svd.matrixV();
	cout << "U=" << U << std::endl;
	cout << "V=" << V << std::endl;

	Eigen::Matrix3d R_ = U * (V.transpose());
	Eigen::Vector3d t_ = Eigen::Vector3d(p1.x, p1.y, p1.z) - R_ * Eigen::Vector3d(p2.x, p2.y, p2.z);

	// convert to cv::Mat
	R = (Mat_<double>(3, 3) <<

		R_(0, 0), R_(0, 1), R_(0, 2),

		R_(1, 0), R_(1, 1), R_(1, 2),

		R_(2, 0), R_(2, 1), R_(2, 2)

		);

	t = (Mat_<double>(3, 1) << t_(0, 0), t_(1, 0), t_(2, 0));

}


