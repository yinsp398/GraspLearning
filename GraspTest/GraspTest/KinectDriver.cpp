
#include "KinectDriver.h"
#include <opencv2\imgproc.hpp>
#include <stdio.h>



KinectDriver::KinectDriver()
{
	m_pDepthInColorFrame = new DepthSpacePoint[COLORWIDTH*COLORHEIGHT];
}

KinectDriver::~KinectDriver()
{
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
	GT_RES res = InitKinect();
	if (res != GT_RES_OK)
		return res;
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

GT_RES	KinectDriver::GetColorImage(cv::Mat *ColorMat)
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
		RGBQUAD *pBuffer = new RGBQUAD[COLORHEIGHT*COLORWIDTH];
		
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
		}
		if (SUCCEEDED(hr))
		{
			hr = RGBConvertMat(pBuffer, COLORWIDTH, COLORHEIGHT, ColorMat);
		}
		if (hr == GT_RES_OK)
		{
			SafeRelease(pColorFrame);
			return GT_RES_OK;
		}
	}
	SafeRelease(pColorFrame);
	return GT_RES_ERROR;
}

GT_RES	KinectDriver::GetDepthImage(cv::Mat *DepthMat, cv::Mat *DepthInColorMat)
{
	HRESULT hr = NULL;
	IDepthFrame *pDepthFrame = NULL;
	if (!m_pDepthFrameReader)
	{
		return GT_RES_NODEVICE;
	}
	hr = m_pDepthFrameReader->AcquireLatestFrame(&pDepthFrame);
	if (SUCCEEDED(hr))
	{
		ColorImageFormat imageFormat = ColorImageFormat_None;
		UINT nBufferSize = 0;
		UINT16 *pBuffer = NULL;
		UINT16 *pBuffer2 = new UINT16[COLORHEIGHT*COLORWIDTH];

		if (SUCCEEDED(hr))
		{
			nBufferSize = DEPTHWIDTH * DEPTHHEIGHT * sizeof(UINT16);
			hr = pDepthFrame->AccessUnderlyingBuffer(&nBufferSize, &pBuffer);
		}
		if (SUCCEEDED(hr))
		{
			hr = DepthConvertMat(pBuffer, DEPTHWIDTH, DEPTHHEIGHT, DepthMat);
		}
		if (hr == GT_RES_OK)
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
					if (x_tmp < 0 || x_tmp >= COLORWIDTH || y_tmp < 0 || y_tmp >= COLORHEIGHT)
						pBuffer2[i*COLORHEIGHT + j] = 0;
					else
						pBuffer2[i*COLORHEIGHT + j] = pBuffer[(x_tmp * DEPTHHEIGHT + y_tmp)];
				}
			}
			hr = DepthConvertMat(pBuffer2, COLORWIDTH, COLORHEIGHT, DepthInColorMat);
			
		}
		if (hr == GT_RES_OK)
		{
			delete []pBuffer2;
			SafeRelease(pDepthFrame);
			return GT_RES_OK;
		}
		delete[]pBuffer2;
	}
	SafeRelease(pDepthFrame);
	return GT_RES_ERROR;
}

//从Kinect获取图像，RGBD四维数据
GT_RES	KinectDriver::GetKinectImage(Graphics *Graph)
{
	GT_RES res;
	cv::Mat ColorMat(COLORHEIGHT, COLORWIDTH, CV_8UC3);
	res = GetColorImage(&ColorMat);
	if (res == GT_RES_OK)
	{
		cvtColor(ColorMat, *(Graph->ColorImg), CV_RGB2GRAY);
	}
	if (res == GT_RES_OK)
	{
		res = GetDepthImage(Graph->DepthImg, Graph->DepthInColorImg);
	}
	return res;
}

GT_RES	KinectDriver::DepthConvertMat(const UINT16* pBuffer, const unsigned int nWidth, const unsigned int nHeight, cv::Mat *pImg)
{
	for (size_t i = 0; i < nHeight; i++)
	{
		for (size_t j = 0; j < nWidth; j++)
		{
			//test use the whole depth data, not only low data.
			pImg->at<UINT16>(i, j) = pBuffer[i*nWidth + j];
		}
	}
	return GT_RES_OK;
}

GT_RES	KinectDriver::RGBConvertMat(const RGBQUAD* pBuffer, const unsigned int nWidth, const unsigned int nHeight, cv::Mat *pImg)
{
	uchar *p_mat = pImg->data;
	const RGBQUAD* pBufferEnd = pBuffer + (nWidth*nHeight);
	while (pBuffer < pBufferEnd)
	{
		*p_mat = pBuffer->rgbBlue;
		p_mat++;
		*p_mat = pBuffer->rgbGreen;
		p_mat++;
		*p_mat = pBuffer->rgbRed;
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