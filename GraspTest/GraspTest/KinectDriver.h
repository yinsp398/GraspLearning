#pragma once
#include "Common.h"
#include <opencv2/core/core.hpp>

struct Graphics;
class  Pose3D;
struct IKinectSensor;
struct IColorFrameReader;
struct IDepthFrameReader;
struct ICoordinateMapper;
typedef struct _CameraSpacePoint CameraSpacePoint;
typedef struct _DepthSpacePoint DepthSpacePoint;
typedef struct _ColorSpacePoint ColorSpacePoint;
typedef struct tagRGBQUAD RGBQUAD;
typedef unsigned short UINT16;

class KinectDriver
{
private:
	IKinectSensor		*		m_pKinectSensor = NULL;								//Kinect传感器指针
	IColorFrameReader	*		m_pColorFrameReader = NULL;							//用于获取彩色帧数据
	IDepthFrameReader	*		m_pDepthFrameReader = NULL;							//用于获取深度图像帧
	ICoordinateMapper	*		m_pCoordinateMapper = NULL;							//用于进行坐标系转换
	UINT16				*		m_pDepthImage = NULL;								//存储当前深度图像（用于进行坐标系转换）
	CameraSpacePoint	*		m_pColorInCameraSpace = NULL;						//彩色图像位置到相机空间的转换矩阵
	DepthSpacePoint		*		m_pColorInDepthSpace = NULL;						//彩色图像位置到深度空间的转换矩阵
public:

	KinectDriver();
	~KinectDriver();

	//启动KinectSensor
	GT_RES	OpenKinect();

	//从Kinect获取图像，RGBD四维数据
	GT_RES	GetKinectImage(Graphics *Graph);

	//关闭KinectSensor
	GT_RES	CloseKinect();

	//根据彩色空间坐标，给出相机空间坐标
	GT_RES	Colorpos2Camerapos(const ColorSpacePoint Colorpos, CameraSpacePoint &Camerapos);
	GT_RES	Colorpos2Camerapos(const std::vector<ColorSpacePoint> Colorpos, std::vector<CameraSpacePoint> &Camerapos);

	//根据彩色空间坐标，给出深度空间坐标
	GT_RES	Colorpos2Depthpos(const ColorSpacePoint Colorpos, DepthSpacePoint &Depthpos);
	GT_RES	Colorpos2Depthpos(const std::vector<ColorSpacePoint> Colorpos, std::vector<DepthSpacePoint> &Depthpos); 
	GT_RES	Colorpos2Depthpos(const float Xc, const float Yc, float &Xd, float &Yd);

	//从彩色图像中的位置得到机器人实际的抓取位置和姿态(x, y, theta)->(x, y, z, Rx, Ry, Rz)
	GT_RES	ColorDepth2Robot(const GraspPose posColor, Pose3D &posUR);

private:

	//获取彩色图像
	GT_RES	GetColorImage(RGBQUAD *ColorImg);

	//获取深度图像
	GT_RES	GetDepthImage(UINT16 *DepthImg);

	//转换深度图到Mat格式
	GT_RES	DepthConvertMat(const UINT16* pBuffer, const unsigned int nWidth, const unsigned int nHeight, cv::Mat *pImg);

	//转换彩色图到Mat格式
	GT_RES	RGBConvertMat(const RGBQUAD* pBuffer, const unsigned int nWidth, const unsigned int nHeight, cv::Mat *pImg);

	// Safe release for interfaces
	template<class Interface>
	inline void SafeRelease(Interface *& pInterfaceToRelease)
	{
		if (pInterfaceToRelease != NULL)
		{
			pInterfaceToRelease->Release();
			pInterfaceToRelease = NULL;
		}
	}
};



