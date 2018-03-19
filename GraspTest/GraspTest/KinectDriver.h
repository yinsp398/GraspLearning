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
	IKinectSensor		*		m_pKinectSensor = NULL;								//Kinect������ָ��
	IColorFrameReader	*		m_pColorFrameReader = NULL;							//���ڻ�ȡ��ɫ֡����
	IDepthFrameReader	*		m_pDepthFrameReader = NULL;							//���ڻ�ȡ���ͼ��֡
	ICoordinateMapper	*		m_pCoordinateMapper = NULL;							//���ڽ�������ϵת��
	UINT16				*		m_pDepthImage = NULL;								//�洢��ǰ���ͼ�����ڽ�������ϵת����
	CameraSpacePoint	*		m_pColorInCameraSpace = NULL;						//��ɫͼ��λ�õ�����ռ��ת������
	DepthSpacePoint		*		m_pColorInDepthSpace = NULL;						//��ɫͼ��λ�õ���ȿռ��ת������
public:

	KinectDriver();
	~KinectDriver();

	//����KinectSensor
	GT_RES	OpenKinect();

	//��Kinect��ȡͼ��RGBD��ά����
	GT_RES	GetKinectImage(Graphics *Graph);

	//�ر�KinectSensor
	GT_RES	CloseKinect();

	//���ݲ�ɫ�ռ����꣬��������ռ�����
	GT_RES	Colorpos2Camerapos(const ColorSpacePoint Colorpos, CameraSpacePoint &Camerapos);
	GT_RES	Colorpos2Camerapos(const std::vector<ColorSpacePoint> Colorpos, std::vector<CameraSpacePoint> &Camerapos);

	//���ݲ�ɫ�ռ����꣬������ȿռ�����
	GT_RES	Colorpos2Depthpos(const ColorSpacePoint Colorpos, DepthSpacePoint &Depthpos);
	GT_RES	Colorpos2Depthpos(const std::vector<ColorSpacePoint> Colorpos, std::vector<DepthSpacePoint> &Depthpos); 
	GT_RES	Colorpos2Depthpos(const float Xc, const float Yc, float &Xd, float &Yd);

	//�Ӳ�ɫͼ���е�λ�õõ�������ʵ�ʵ�ץȡλ�ú���̬(x, y, theta)->(x, y, z, Rx, Ry, Rz)
	GT_RES	ColorDepth2Robot(const GraspPose posColor, Pose3D &posUR);

private:

	//��ȡ��ɫͼ��
	GT_RES	GetColorImage(RGBQUAD *ColorImg);

	//��ȡ���ͼ��
	GT_RES	GetDepthImage(UINT16 *DepthImg);

	//ת�����ͼ��Mat��ʽ
	GT_RES	DepthConvertMat(const UINT16* pBuffer, const unsigned int nWidth, const unsigned int nHeight, cv::Mat *pImg);

	//ת����ɫͼ��Mat��ʽ
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



