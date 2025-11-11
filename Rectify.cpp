#include "Rectify.h"
#include <regex>
#include <map>

#include <pcl/io/ply_io.h>
#include <pcl/io/io.h>
//#include <pcl/io/pcd_io.h>

using namespace cv;
//using namespace std;
//using namespace pcl;

# define MyMIN(a,b)  ((a) > (b) ? (b) : (a))
#define MyMAX(a,b)  ((a) < (b) ? (b) : (a))
//读入标定参数，


bool Rectify::calib(const string &intrinsic, const string & extrinsic, int W, int H)
{

	imgSize = Size(W, H);
    // 01 读取标定文件
	// 1 加载内参
	load_intrinsic(intrinsic);

    //load_rectify(rectify);

	// 2 加载外参
	load_extrinsic(extrinsic);

	//立体校正相关程序
	stereoRectify(
		M_L, D_L,
		M_R, D_R,
		imgSize,
		R, T,
		rt_L, rt_R,
		P_L, P_R,
		Q,                             //Q 输出4x4的视差-深度映射矩阵
		0,//CALIB_ZERO_DISPARITY,
		-1,                            // alpha 自由的尺度参数。如果是-1或缺席，函数将执行默认的缩放。否则，参数应该在0和1之间。alpha=0表示校正后的图像被缩放和移动，只有有效像素可见(校正后没有黑色区域)。alpha=1表示对校正后的图像进行抽取和移位，使所有来自相机的原始图像的像素都保留在校正后的图像中(无源图像像素丢失)。任何中间值都是这两个极端情况的中间结果。

		imgSize,                       //校正后图像分辨率
		&validROI_L, &validROI_R);     //左右图像的有效区域,矫正图像内的可选的输出矩形，其中所有像素是有效的。如果alpha=0, ROI覆盖整个图像。否则，它们可能会更小


    //计算去畸变映射矩阵，mapx_L, mapy_L
	initUndistortRectifyMap(M_L, D_L, rt_L, P_L,
		imgSize, CV_16SC2, mapx_L, mapy_L);


	//计算去畸变映射矩阵，mapx_R, mapy_R
	initUndistortRectifyMap(M_R, D_R, rt_R, P_R,
		imgSize, CV_16SC2, mapx_R, mapy_R);
        

    return true;
}



bool Rectify::calcDistance(cv::Mat& dif_map, cv::Mat& depth_map, vector<pair<cv::Point2f, cv::Point2f>>& cps, float min_z, float max_z) {

    for (auto& cp : cps) {
        cv::Point2f l = cp.first, r = cp.second;
        float d = l.x - r.x;
        if (d < 0) {
            d = 0.0;
        }
        dif_map.at<float>(int(l.y), int(l.x)) = d;
    }

    // depth_map: 映射后存储三维坐标的图像
    cv::reprojectImageTo3D(dif_map, depth_map, Q, false, CV_32F);
    // 处理异常值
    for (int row = 0; row < depth_map.rows; ++row)
    {
        for (int col = 0; col < depth_map.cols; ++col)
        {
            float z = depth_map.at<cv::Vec3f>(row, col)[2];
            if (z < min_z || z > max_z || dif_map.at<float>(row, col) <= 0)
            {
                depth_map.at<cv::Vec3f>(row, col) = cv::Vec3f(0, 0, 0);
            }
        }
    }

    return true;
}




bool Rectify::rectify(cv::Mat &imgL_src, cv::Mat &imgR_src, cv::Mat &imgL_des, cv::Mat &imgR_des)
{
    // 单目畸变校正
    cv::undistort(imgL_src, imgL_des, M_L, D_L);
    cv::undistort(imgR_src, imgR_des, M_R, D_R);
    // 双目畸变校正
    cv::remap(imgL_des, imgL_des, mapx_L, mapy_L, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
    cv::remap(imgR_des, imgR_des, mapx_R, mapy_R, cv::INTER_LINEAR, cv::BORDER_CONSTANT);


    
	bool showRect = false;
	Mat img1r, img2r;

	if (showRect)
	{
		Mat canvas;
		double sf;
		int w, h;
		sf = 600. / MyMAX(imgSize.width, imgSize.height);  //600
		w = cvRound(imgSize.width*sf);
		h = cvRound(imgSize.height*sf);
		canvas.create(h, w * 2, CV_8UC3);

		//cvtColor(imgL_des, img1r, COLOR_GRAY2BGR);
		//cvtColor(imgR_des, img2r, COLOR_GRAY2BGR);
		img1r = imgL_des;
		img2r = imgR_des;

		Mat canvasPart = canvas(Rect(w * 0, 0, w, h));
		resize(img1r, canvasPart, canvasPart.size(), 0, 0, INTER_AREA);
		canvasPart = canvas(Rect(w * 1, 0, w, h));
		resize(img2r, canvasPart, canvasPart.size(), 0, 0, INTER_AREA);

		for (int j = 0; j < canvas.rows; j += 16)
			line(canvas, Point(0, j), Point(canvas.cols, j), Scalar(0, 255, 0), 1, 8);

		imshow("rectified", canvas);
		//waitKey(3000);
		//destroyWindow("rectified");
		char c = (char)waitKey();
		if (c == 27 || c == 'q' || c == 'Q')
			destroyWindow("rectified");


	}
	

    return true;

}



bool Rectify::calcDistance2(cv::Mat& dif_map, cv::Mat& depth_map, cv::Mat& mask, float min_z, float max_z)
{


    // depth_map: 映射后存储三维坐标的图像
    cv::reprojectImageTo3D(dif_map, depth_map, Q, false, CV_32F);
    
    // 处理异常值
    for (int row = 0; row < depth_map.rows; ++row)
    {
        for (int col = 0; col < depth_map.cols; ++col)
        {
            float z = depth_map.at<cv::Vec3f>(row, col)[2];
            if (z < min_z || z > max_z || dif_map.at<short>(row, col) <= 10||mask.at<uchar>(row,col)==0)
            {
                depth_map.at<cv::Vec3f>(row, col) = cv::Vec3f(0, 0, 0);
            }
        }
    }

    return true;
}



bool Rectify::saveXYZ(const string& filename, cv::Mat& depth_map)
{
    const double max_z = 1.0e4;
    FILE* fp = fopen(filename.data(), "wt");
    for (int y = 0; y < depth_map.rows; y++)
    {
        for (int x = 0; x < depth_map.cols; x++)
        {
            //float point_x = depth_map.at<cv::Vec3f>(y, x)[0];
            //float point_y = depth_map.at<cv::Vec3f>(y, x)[1];
            float point_z = depth_map.at<cv::Vec3f>(y, x)[2];

            //fprintf(fp, "%.1f %.1f %.1f\n", y, x, point_z);
            fprintf(fp, "%.1f ", point_z);

            /*
            if (depth_map.at<cv::Vec3f>(y, x) == cv::Vec3f(0, 0, 0))
            {
               
            }
            else
            {
                fprintf(fp, "%.1f %.1f %.1f\n", point_x, point_y, point_z);
            }
            */

        }
        fprintf(fp, "\n");
        fprintf(fp, "\n");

    }
    fclose(fp);
    cout << "写入点云数据到文件:" << filename << std::endl;
    return true;
}




bool Rectify::savePLY(const string& filename, cv::Mat& depth_map)
{

    const double max_z = 1.0e4;
   
    pcl::PointCloud<pcl::PointXYZ> cloud_a;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_a.height = depth_map.rows;
    cloud_a.width = depth_map.cols;
    cloud_a.points.resize(cloud_a.width * cloud_a.height);

    int num = 0;
    for (int y = 0; y < depth_map.rows; y++)
    {
        for (int x = 0; x < depth_map.cols; x++)
        {
            float point_x = depth_map.at<cv::Vec3f>(y, x)[0];
            float point_y = depth_map.at<cv::Vec3f>(y, x)[1];
            float point_z = depth_map.at<cv::Vec3f>(y, x)[2];

            //if (fabs(point_z) >max_z || fabs(point_x) >max_z || fabs(point_y) >max_z)
            if (depth_map.at<cv::Vec3f>(y, x) == cv::Vec3f(0, 0, 0))
            {
               // point_x = 0;
                //point_y = 0;
                //point_z = 0;
                //cloud_a.points[num].x = point_x;
                //cloud_a.points[num].y = point_y;
                //cloud_a.points[num].z = point_z;
               
            }
            else
            {
                cloud_a.points[num].x = point_x;
                cloud_a.points[num].y = point_y;
                cloud_a.points[num].z = point_z;
               
            }
            num++;

        }
    }
  
    *cloud = cloud_a;
    pcl::PLYWriter writer;
    writer.write(filename, *cloud, true);
   

    return true;
}



