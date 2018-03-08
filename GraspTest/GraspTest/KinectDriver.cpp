
#include "KinectDriver.h"
#include <stdio.h>


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

GT_RES	KinectDriver::GetColorImage(RGBQUAD *ColorImg)
{
	HRESULT hr = NULL;
	IColorFrame *pColorFrame = NULL;
	if (!m_pColorFrameReader)
	{
		return GT_RES_NODEVICE;
	}
	hr = m_pColorFrameReader->AcquireLatestFrame(&pColorFrame);
	if (SUCCEEDED(hr))
	{
		IFrameDescription * pFrameDescription = NULL;
		ColorImageFormat imageFormat = ColorImageFormat_None;
		UINT nBufferSize = 0;
		RGBQUAD *pBuffer = ColorImg;
		
		if (SUCCEEDED(hr))
		{
			hr = pColorFrame->get_RawColorImageFormat(&imageFormat);
		}
		if (SUCCEEDED(hr))
		{
			if (imageFormat == ColorImageFormat_Bgra)
			{
				hr = pColorFrame->AccessRawUnderlyingBuffer(&nBufferSize, reinterpret_cast<BYTE**> (&pBuffer));
			}
			else
			{
				nBufferSize = COLORWIDTH * COLORHEIGHT * sizeof(RGBQUAD);
				hr = pColorFrame->CopyConvertedFrameDataToArray(nBufferSize, reinterpret_cast<BYTE*>(pBuffer), ColorImageFormat_Bgra);
			}
			return GT_RES_OK;
		}
	}
	SafeRelease(pColorFrame);
	return GT_RES_ERROR;
}

GT_RES	KinectDriver::GetDepthImage(UINT16 *DepthImg, UINT16 *DepthInColorImg)
{
	HRESULT hr = NULL;
	IDepthFrame *pDepthFrame = NULL;
	if (!m_pColorFrameReader)
	{
		return GT_RES_NODEVICE;
	}
	hr = m_pDepthFrameReader->AcquireLatestFrame(&pDepthFrame);
	if (SUCCEEDED(hr))
	{
		ColorImageFormat imageFormat = ColorImageFormat_None;
		UINT nBufferSize = 0;
		UINT16 *pBuffer = DepthImg;
		UINT16 *pBuffer2 = DepthInColorImg;

		if (SUCCEEDED(hr))
		{
			nBufferSize = DEPTHWIDTH * DEPTHHEIGHT * sizeof(UINT16);
			hr = pDepthFrame->CopyFrameDataToArray(nBufferSize, pBuffer);
		}
		if (SUCCEEDED(hr))
		{
			hr = m_pCoordinateMapper->MapColorFrameToDepthSpace(DEPTHHEIGHT*DEPTHWIDTH, pBuffer, COLORWIDTH*COLORHEIGHT, m_pDepthInColorFrame);
		}
		if (SUCCEEDED(hr))
		{
			for (int i = 0; i < COLORWIDTH; i++)
			{
				for (int j = 0; j < COLORHEIGHT; j++)
				{
					int x_tmp = m_pDepthInColorFrame[i*COLORHEIGHT + j].X;					//XandY is float ,so maybe we can use interpolation to make it more percise
					int y_tmp = m_pDepthInColorFrame[i*COLORHEIGHT + j].Y;
					pBuffer2[i*COLORHEIGHT + j] = pBuffer[(x_tmp * DEPTHHEIGHT + y_tmp)];
				}
			}
			return GT_RES_OK;
		}
	}

	SafeRelease(pDepthFrame);
	return GT_RES_ERROR;
}

//从Kinect获取图像，RGBD四维数据
GT_RES	KinectDriver::GetKinectImage(Graphics *Graph)
{
	GT_RES res;
	RGBQUAD ColorImg[COLORHEIGHT*COLORWIDTH];
	BYTE Gray[COLORHEIGHT*COLORWIDTH];
	UINT16 DepthImg[DEPTHHEIGHT*DEPTHWIDTH];
	UINT16 DepthInColorImg[COLORHEIGHT*COLORWIDTH];
	res = GetColorImage(ColorImg);
	if (res != GT_RES_OK)
	{
		res = GetDepthImage(DepthImg, DepthInColorImg);
	}
	if (res != GT_RES_OK)
	{
		res = DepthConvertMat(DepthImg, Graph->DepthImg);
	}
	if (res != GT_RES_OK)
	{
		res = DepthConvertMat(DepthInColorImg, Graph->DepthInColorImg);
	}
	if (res != GT_RES_OK)
	{
		res = RGBAConvertG(Gray, ColorImg, COLORWIDTH, COLORHEIGHT);
	}
	if (res != GT_RES_OK)
	{
		res = RGBConvertMat(Gray, Graph->ColorImg);
	}
	return res;
}

GT_RES	KinectDriver::DepthConvertMat(const UINT16* pBuffer, cv::Mat *pImg)
{
	uchar * p_mat = pImg->data;
	const UINT16* pBufferEnd = pBuffer + (DEPTHHEIGHT*DEPTHWIDTH);
	while (pBuffer < pBufferEnd)
	{
		USHORT depth = *pBuffer;
		BYTE intensity = static_cast<BYTE>(depth % 256);					
		//map depth to 0~255,but need to notice that when the object is on the distance of 256mm*K,it will introduce some error.
		*p_mat = intensity;
		p_mat++;
		++pBuffer;
	}
	return GT_RES_OK;
}

GT_RES	KinectDriver::RGBConvertMat(const BYTE* pBuffer, cv::Mat *pImg)
{
	uchar *p_mat = pImg->data;
	const BYTE* pBufferEnd = pBuffer + (COLORWIDTH*COLORHEIGHT);
	while (pBuffer < pBufferEnd)
	{
		*p_mat = *pBuffer;
		p_mat++;
		++pBuffer;
	}
	return GT_RES_OK;
}

//彩色图像转化为灰度图
GT_RES	KinectDriver::RGBAConvertG(BYTE *pGray, const RGBQUAD *pBuffer, const unsigned int Width, const unsigned int Height)
{
	for (unsigned int i = 0; i < Width*Height; i++)
	{
		pGray[i] = (BYTE)(pBuffer[i].rgbBlue/3.0 + pBuffer[i].rgbGreen/3.0 + pBuffer[i].rgbRed/3.0);
	}
	return GT_RES_OK;
}