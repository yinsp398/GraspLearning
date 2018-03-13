#pragma once
#include "Common.h"

class Pose3D;
typedef unsigned short UINT16;

class CoordinateMap
{
private:
	ICoordinateMapper *		m_pKinectCoordinate;
	UINT16			  *		m_pDepthBuffer;
public:

	CoordinateMap(ICoordinateMapper *KinectCoordinate);
	~CoordinateMap();

	//从相机坐标系下的坐标转换到机器人坐标系下坐标，从而确定目标点的位置
	GT_RES	Robot2Camera(Pose3D *posCamera, Pose3D *posUR);

	//从相机坐标系获取其在深度图像中的位置
	GT_RES	Camera2Depth();

	//从相机坐标系的位置获取其在彩色图像中的位置
	GT_RES	Camera2Color();

	//从彩色图像中的位置获取其在深度图像中的位置
	GT_RES	Color2Depth(const unsigned int Xc, const unsigned int Yc, unsigned int &Xd, unsigned int &Yd);

	//从彩色图像中的位置得到机器人实际的抓取位置和姿态(x,y,theta)->(x,y,z,Rx,Ry,Rz)
	GT_RES	ColorDepth2Robot(const GraspPose posColor, Pose3D &posUR);
};