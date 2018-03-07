
#include "KinectDriver.h"
#include <stdio.h>

struct Graphics
{
	RGBQUAD *Rgba;
	UINT16	*depth;
	UINT16	*depth2;
};

KinectDriver::KinectDriver()
{
	InitKinect();
	m_pDepthInColorFrame = new DepthSpacePoint[COLORWIDTH*COLORHEIGHT];
}
KinectDriver::~KinectDriver()
{
	CloseKinect();
	UInitKinect();
	delete[] m_pDepthInColorFrame;
}

//关闭KinectSensor
GT_RES	KinectDriver::CloseKinect()
{
	HRESULT hr;
	SafeRelease(m_pDepthFrameReader);
	SafeRelease(m_pColorFrameReader);
	hr = m_pKinectSensor->Close();
	if (!SUCCEEDED(hr))
	{
		return GT_RES_ERROR;
	}
	if (UInitKinect() != GT_RES_OK)
	{
		return GT_RES_ERROR;
	}
	return GT_RES_OK;
}

//释放Kinect
GT_RES	KinectDriver::UInitKinect()
{
	SafeRelease(m_pKinectSensor);
	return GT_RES_OK;
}

//启动KinectSensor
GT_RES	KinectDriver::OpenKinect()
{
	IColorFrameSource *pColorFrameSource = NULL;
	IDepthFrameSource *pDepthFrameSource = NULL;
	HRESULT hr;
	BOOLEAN	Available = false;
	if (m_pKinectSensor)
	{
		/*
		hr = m_pKinectSensor->get_IsAvailable(&Available);
		if (!Available)
		{
			printf("KinectSensor is not availabel!\n");
			return GT_RES_NODEVICE;
		}
		Available = false;
		*/
		hr = m_pKinectSensor->Open();
		if (SUCCEEDED(hr))
		{
			hr = m_pKinectSensor->get_IsOpen(&Available);
		}
		if (!Available)
		{
			return GT_RES_DEVICEERROR;
		}
		if (SUCCEEDED(hr))
		{
			hr = m_pKinectSensor->get_CoordinateMapper(&m_pCoordinateMapper);
		}
		if (SUCCEEDED(hr))
		{
			hr = m_pKinectSensor->get_ColorFrameSource(&pColorFrameSource);
		}
		if (SUCCEEDED(hr))
		{
			hr = pColorFrameSource->OpenReader(&m_pColorFrameReader);
		}
		if (SUCCEEDED(hr))
		{
			hr = m_pKinectSensor->get_DepthFrameSource(&pDepthFrameSource);
		}
		if (SUCCEEDED(hr))
		{
			hr = pDepthFrameSource->OpenReader(&m_pDepthFrameReader);
		}
		SafeRelease(pColorFrameSource);
		SafeRelease(pDepthFrameSource);
	}
	if (!m_pKinectSensor || FAILED(hr))
	{
		printf("No ready Kinect found!\n");
		return GT_RES_NODEVICE;
	}
	return GT_RES_OK;
}

//初始化Kinect
GT_RES	KinectDriver::InitKinect()
{
	//Initialize sensor
	HRESULT hr;
	hr = GetDefaultKinectSensor(&m_pKinectSensor);
	if (FAILED(hr))
	{
		printf("No ready Kinect.\n");
		return GT_RES_NODEVICE;
	}

	return GT_RES_OK;
}


//从Kinect获取图像，RGBD四维数据
GT_RES	KinectDriver::GetKinectImage(Graphics *Graph)
{
	IColorFrame * pColorFrame = NULL;
	IDepthFrame * pDepthFrame = NULL;
	HRESULT hr = NULL;
	if (!m_pColorFrameReader || !m_pDepthFrameReader)
	{
		return GT_RES_NODEVICE;
	}
	hr = m_pColorFrameReader->AcquireLatestFrame(&pColorFrame);
	if (SUCCEEDED(hr))
	{
		hr = m_pDepthFrameReader->AcquireLatestFrame(&pDepthFrame);
	}
	if (SUCCEEDED(hr))
	{
		INT64 nTime = 0;
		IFrameDescription * pFrameDescription = NULL;
		int nWidth;
		int nHeigh;
		USHORT nDepthMaxDistance;
		USHORT nDepthMinDistance;
		ColorImageFormat imageFormat = ColorImageFormat_None;
		UINT nBufferSize = 0;
		RGBQUAD *pBuffer1 = NULL;
		UINT16 *pBuffer2 = NULL;
		UINT16 *pBuffer3 = NULL;

		hr = pColorFrame->get_RelativeTime(&nTime);
		if (SUCCEEDED(hr))
		{
			hr = pColorFrame->get_FrameDescription(&pFrameDescription);
		}
		if (SUCCEEDED(hr))
		{
			hr = pFrameDescription->get_Width(&nWidth);
		}
		if (SUCCEEDED(hr))
		{
			hr = pFrameDescription->get_Height(&nHeigh);
		}
		if (SUCCEEDED(hr))
		{
			hr = pColorFrame->get_RawColorImageFormat(&imageFormat);
		}
		if (SUCCEEDED(hr))
		{
			if (imageFormat == ColorImageFormat_Bgra)
			{
				hr = pColorFrame->AccessRawUnderlyingBuffer(&nBufferSize, reinterpret_cast<BYTE**> (&pBuffer1));
			}
			else
			{
				pBuffer1 = Graph->Rgba;
				nBufferSize = COLORWIDTH * COLORHEIGHT * sizeof(RGBQUAD);
				hr = pColorFrame->CopyConvertedFrameDataToArray(nBufferSize, reinterpret_cast<BYTE*>(pBuffer1), ColorImageFormat_Bgra);
			}
		}
		if (SUCCEEDED(hr))
		{
			hr = pDepthFrame->get_FrameDescription(&pFrameDescription);
		}
		if (SUCCEEDED(hr))
		{
			hr = pFrameDescription->get_Width(&nWidth);
		}
		if (SUCCEEDED(hr))
		{
			hr = pFrameDescription->get_Height(&nHeigh);
		}
		if (SUCCEEDED(hr))
		{
			hr = pDepthFrame->get_DepthMaxReliableDistance(&nDepthMaxDistance);
		}
		if (SUCCEEDED(hr))
		{
			hr = pDepthFrame->get_DepthMinReliableDistance(&nDepthMinDistance);
		}
		if (SUCCEEDED(hr))
		{
			pBuffer2 = Graph->depth;
			nBufferSize = DEPTHWIDTH * DEPTHHEIGHT * sizeof(UINT16);
			hr = pDepthFrame->CopyFrameDataToArray(nBufferSize, pBuffer2);
		}
		if (SUCCEEDED(hr))
		{
			hr = m_pCoordinateMapper->MapColorFrameToDepthSpace(DEPTHHEIGHT*DEPTHWIDTH, pBuffer2, COLORWIDTH*COLORHEIGHT, m_pDepthInColorFrame);
		}
		if (SUCCEEDED(hr))
		{
			pBuffer3 = Graph->depth2;
			for (int i = 0; i < COLORWIDTH; i++)
			{
				for (int j = 0; j < COLORHEIGHT; j++)
				{
					int x_tmp = m_pDepthInColorFrame[i*COLORHEIGHT + j].X;					//XandY is float ,so maybe we can use interpolation to make it more percise
					int y_tmp = m_pDepthInColorFrame[i*COLORHEIGHT + j].Y;
					pBuffer3[i*COLORHEIGHT + j] = pBuffer2[(x_tmp * DEPTHHEIGHT + y_tmp)];
				}
			}
		}

	}
	SafeRelease(pColorFrame);
	SafeRelease(pDepthFrame);
	return GT_RES_DEVICEERROR;
}

#if 0
//彩色图像转化为灰度图
GT_RES	KinectDriver::RGBA2G(int Width, int Heigh, Graphics *Graph, RGBQUAD *pBuffer)
{
	if (Width != COLORWIDTH || Heigh != COLORHEIGHT)
	{
		return GT_RES_ERROR;
	}
	Graph->gray = new BYTE[Width*Heigh];
	Graph->alpha = new BYTE[Width*Heigh];
	for (int i = 0; i < Width*Heigh; i++)
	{
		Graph->gray[i] = (BYTE)(pBuffer[i].rgbBlue/3.0 + pBuffer[i].rgbGreen/3.0 + pBuffer[i].rgbRed/3.0);
		Graph->alpha[i] = pBuffer[i].rgbReserved;
	}
	return GT_RES_OK;
}
#endif