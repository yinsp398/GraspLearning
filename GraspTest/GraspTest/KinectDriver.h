#pragma once
#include "Common.h"
#include <Kinect.h>

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
#if 0
	//��ɫͼ��ת��Ϊ�Ҷ�ͼ
	GT_RES	RGBA2G(int Width, int Heigh, Graphics *Graph, RGBQUAD *pBuffer);
#endif
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



