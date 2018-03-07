
#include "CoordinateMap.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "utils.h"



CoordinateMap::CoordinateMap()
{

}
CoordinateMap::~CoordinateMap()
{

}

GT_RES	CoordinateMap::Robot2Camera(Pose3D *posCamera, Pose3D *posUR)
{
	return GT_RES_OK;
}

GT_RES	CoordinateMap::Camera2Depth()
{
	return GT_RES_OK;
}

GT_RES	CoordinateMap::Color2Depth(const unsigned int Xc, const unsigned int Yc, unsigned int &Xd, unsigned int &Yd)
{
	return GT_RES_OK;
}

GT_RES	CoordinateMap::ColorDepth2Robot(const GraspPose posColor, const float Depth, Pose3D &posUR)
{
	cv::Mat MatColor(cv::Size(1, 4), CV_32F);
	cv::Mat MatUR(cv::Size(1, 4), CV_32F);
	cv::Mat MatTrans(cv::Size(3, 3), CV_32F);
	MatColor.at<float>(0, 0) = posColor.x;
	MatColor.at<float>(1, 0) = posColor.y;
	MatColor.at<float>(2, 0) = Depth;
	MatColor.at<float>(3, 0) = 1;
	MatTrans.at<float>(0, 0) = COLOR2ROBOT11;
	MatTrans.at<float>(0, 1) = COLOR2ROBOT12;
	MatTrans.at<float>(0, 2) = COLOR2ROBOT13;
	MatTrans.at<float>(0, 3) = COLOR2ROBOT14;
	MatTrans.at<float>(1, 0) = COLOR2ROBOT21;
	MatTrans.at<float>(1, 1) = COLOR2ROBOT22;
	MatTrans.at<float>(1, 2) = COLOR2ROBOT23;
	MatTrans.at<float>(1, 3) = COLOR2ROBOT24;
	MatTrans.at<float>(2, 0) = COLOR2ROBOT31;
	MatTrans.at<float>(2, 1) = COLOR2ROBOT32;
	MatTrans.at<float>(2, 2) = COLOR2ROBOT33;
	MatTrans.at<float>(2, 3) = COLOR2ROBOT34;
	MatTrans.at<float>(3, 0) = 0;
	MatTrans.at<float>(3, 1) = 0;
	MatTrans.at<float>(3, 2) = 0;
	MatTrans.at<float>(3, 3) = 1;
	MatUR = MatTrans*MatColor;

	posUR.x = MatUR.at<float>(0, 0);
	posUR.y = MatUR.at<float>(1, 0);
	posUR.z = MatUR.at<float>(2, 0);
	posUR.Rx = PI;
	posUR.Ry = 0;
	posUR.Rz = posColor.theta + ANGLEBIAS;
	return GT_RES_OK;
}