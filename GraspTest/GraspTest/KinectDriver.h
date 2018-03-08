#pragma once
#include "Common.h"
#include <Kinect.h>
#include <opencv2/core/core.hpp>
struct Graphics;


class KinectDriver
{
private:
	IKinectSensor *			m_pKinectSensor = NULL;
	IColorFrameReader *		m_pColorFrameReader = NULL;
	IDepthFrameReader *		m_pDepthFrameReader = NULL;
	ICoordinateMapper *		m_pCoordinateMapper = NULL;
	DepthSpacePoint	*		m_pDepthInColorFrame = NULL;
public:

	KinectDriver();
	~KinectDriver();

	//����KinectSensor
	GT_RES	OpenKinect();

	//��Kinect��ȡͼ��RGBD��ά����
	GT_RES	GetKinectImage(Graphics *Graph);

	//�ر�KinectSensor
	GT_RES	CloseKinect();

private:
	//��ʼ��KinectSensor
	GT_RES	InitKinect();

	//�ͷ�Kinect
	GT_RES	UInitKinect();

	//��ȡ��ɫͼ��
	GT_RES	GetColorImage(RGBQUAD *ColorImg);

	//��ȡ���ͼ��
	GT_RES	GetDepthImage(UINT16 *DepthImg, UINT16 *DepthInColorImg);

	//ת�����ͼ��Mat��ʽ
	GT_RES	DepthConvertMat(const UINT16* pBuffer, cv::Mat *pImg);

	//ת����ɫͼ��Mat��ʽ
	GT_RES	RGBConvertMat(const BYTE* pBuffer, cv::Mat *pImg);

	//ת����ɫͼ���Ҷ�ͼ
	GT_RES	RGBAConvertG(BYTE *pGray, const RGBQUAD *pBuffer, const unsigned int Width, const unsigned int Height);

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



