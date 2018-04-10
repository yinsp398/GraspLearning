
#include "KinectDriver.h"
#include <utils.h>
#include <opencv2\imgproc.hpp>
#include <Kinect.h>
#include <numeric>
#include <utility>
#include <stdio.h>



KinectDriver::KinectDriver()
{
	OpenKinect();
}

KinectDriver::~KinectDriver()
{
	if (m_pDepthImage)
	{
		delete[]m_pDepthImage;
		m_pDepthImage = NULL;
	}
	CloseKinect();
}

GT_RES	KinectDriver::CloseKinect()
{
	HRESULT hr;
	SafeRelease(m_pDepthFrameReader);
	SafeRelease(m_pColorFrameReader);
	hr = m_pKinectSensor->Close();
	SafeRelease(m_pKinectSensor);
	if (!SUCCEEDED(hr))
	{
		return GT_RES_ERROR;
	}
	return GT_RES_OK;
}

GT_RES	KinectDriver::OpenKinect()
{
	HRESULT hr;
	IColorFrameSource *pColorFrameSource = NULL;
	IDepthFrameSource *pDepthFrameSource = NULL;
	BOOLEAN	Available = false;

	hr = GetDefaultKinectSensor(&m_pKinectSensor);
	if (FAILED(hr))
	{
		printf("No ready Kinect.\n");
		return GT_RES_NODEVICE;
	}
	if (m_pKinectSensor)
	{
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
			nBufferSize = COLORWIDTH * COLORHEIGHT * sizeof(RGBQUAD);
			hr = pColorFrame->CopyConvertedFrameDataToArray(nBufferSize, reinterpret_cast<BYTE*>(pBuffer), ColorImageFormat_Bgra);
		}
		if (SUCCEEDED(hr))
		{
			SafeRelease(pColorFrame);
			return GT_RES_OK;
		}
	}
	SafeRelease(pColorFrame);
	return GT_RES_ERROR;
}

GT_RES	KinectDriver::GetDepthImage(UINT16 *DepthImg)
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

		if (SUCCEEDED(hr))
		{
			nBufferSize = DEPTHWIDTH * DEPTHHEIGHT * sizeof(UINT16);
			hr = pDepthFrame->AccessUnderlyingBuffer(&nBufferSize, &pBuffer);
		}
		if (SUCCEEDED(hr))
		{
			memcpy(DepthImg, pBuffer, DEPTHWIDTH * DEPTHHEIGHT * sizeof(UINT16));
			SafeRelease(pDepthFrame);
			return GT_RES_OK;
		}
	}
	SafeRelease(pDepthFrame);
	return GT_RES_ERROR;
}

GT_RES	KinectDriver::GetKinectImage(Graphics *Graph)
{
	GT_RES res;
	RGBQUAD *ColorImg = new RGBQUAD[COLORWIDTH*COLORHEIGHT];
	if (m_pDepthImage)
	{
		delete[]m_pDepthImage;
		m_pDepthImage = NULL;
	}
	if (m_pColorInDepthSpace)
	{
		delete[]m_pColorInDepthSpace;
		m_pColorInDepthSpace = NULL;
	}
	m_pDepthImage = new UINT16[DEPTHWIDTH*DEPTHHEIGHT];
	UINT16 *DepthImg = m_pDepthImage;
	float *DepthSum = new float[DEPTHWIDTH*DEPTHHEIGHT];
	cv::Mat ColorMat(COLORHEIGHT, COLORWIDTH, CV_8UC3);
	//Get and deal Color Img
	res = GetColorImage(ColorImg);
	if (res != GT_RES_OK)
	{
		printf("GetColorImg failed!\n");
		delete[]ColorImg;
		return res;
	}
	res = RGBConvertMat(ColorImg, COLORWIDTH, COLORHEIGHT, &ColorMat);
	if (res != GT_RES_OK)
	{
		printf("RGBConvertMat failed!\n");
		delete[]ColorImg;
		return res;
	}
	cvtColor(ColorMat, *(Graph->ColorImg), CV_RGB2GRAY);
	//Get several DepthImg and get average
	for (int i = 0; i < DEPTHMEANCNT; i++)
	{
		res = GetDepthImage(DepthImg);
		if (res != GT_RES_OK)
		{
			printf("GetDepthImg failed!\n");
			delete[]ColorImg;
			return res;
		}
		for (int j = 0; j < DEPTHWIDTH*DEPTHHEIGHT; j++)
		{
			DepthSum[j] += (float)(DepthImg[j]) / DEPTHMEANCNT;
		}
		Sleep(100);
	}
	for (int j = 0; j < DEPTHWIDTH*DEPTHHEIGHT; j++)
	{
		DepthImg[j] = (UINT16)(DepthImg[j] + 0.5);
	}
	if (Graph->DepthImg)
	{
		res = DepthConvertMat(DepthImg, DEPTHWIDTH, DEPTHHEIGHT, Graph->DepthImg);
		if (res != GT_RES_OK)
		{
			printf("DepthConvertMat failed!\n");
			delete[]ColorImg;
			return res;
		}
	}
	//Get CloudPointsImg
	//Get CameraSpacePoint map to Colorframe
	if (m_pColorInCameraSpace)
	{
		delete[]m_pColorInCameraSpace;
		m_pColorInCameraSpace = NULL;
	}
	m_pColorInCameraSpace = new CameraSpacePoint[COLORHEIGHT*COLORWIDTH];
	HRESULT hr = m_pCoordinateMapper->MapColorFrameToCameraSpace(DEPTHWIDTH*DEPTHHEIGHT, DepthImg, COLORWIDTH*COLORHEIGHT, m_pColorInCameraSpace);
	if (FAILED(hr))
	{
		printf("MapColorFrameToCameraSpace failed!\n");
		return GT_RES_ERROR;
	}
	//Get point x and y of edge
	size_t ColorLeft, ColorRight, ColorUp, ColorDown;
	ColorLeft = max(COLORSPACELEFT - COLORGRAPHWIDTH, 0);
	ColorRight = min(COLORSPACERIGHT + COLORGRAPHWIDTH, COLORWIDTH - 1);
	ColorUp = max(COLORSPACEUP - COLORGRAPHHEIGHT, 0);
	ColorDown = min(COLORSPACEDOWN + COLORGRAPHHEIGHT, COLORHEIGHT - 1);
	CameraSpacePoint *LeftUp, *LeftDown, *RightUp, *RightDown;
	LeftUp = &m_pColorInCameraSpace[ColorUp*COLORWIDTH + ColorLeft];
	LeftDown = &m_pColorInCameraSpace[ColorDown*COLORWIDTH + ColorLeft];
	RightUp = &m_pColorInCameraSpace[ColorUp*COLORWIDTH + ColorRight];
	RightDown = &m_pColorInCameraSpace[ColorDown*COLORWIDTH + ColorRight];
	int CameraLeft, CameraRight, CameraUp, CameraDown;
	CameraLeft = (int)(min(LeftUp->X, LeftDown->X)*1000+1);
	CameraRight = (int)(max(RightUp->X, RightDown->X) * 1000);
	CameraUp = (int)(max(LeftUp->Y, RightUp->Y) * 1000);
	CameraDown = (int)(min(LeftDown->Y, RightDown->Y) * 1000+1);
	m_CloudHeightBias = CameraDown;
	m_CloudWidthBias = CameraLeft;
	m_CloudHeight = (CameraUp - CameraDown) / CLOUDRESOLUTION;
	m_CloudWidth = (CameraRight - CameraLeft) / CLOUDRESOLUTION;
	std::vector<std::vector<std::vector<CameraSpacePoint> > > V_CloudPoints;
	V_CloudPoints.resize(m_CloudHeight);
	for (size_t i = 0; i < m_CloudHeight; i++)
	{
		V_CloudPoints[i].resize(m_CloudWidth);
	}
	//float MinDepth = FLT_MAX, MaxDepth = -FLT_MAX;
	for (size_t i = ColorUp; i <= ColorDown; i++)
	{
		for (size_t j = ColorLeft; j <= ColorRight; j++)
		{
			CameraSpacePoint *tmp = &m_pColorInCameraSpace[i*COLORWIDTH + j];
			int tmpY = int((tmp->Y * 1000 - CameraDown)/CLOUDRESOLUTION + 0.5);
			int tmpX = int((tmp->X * 1000 - CameraLeft) / CLOUDRESOLUTION + 0.5);
			if (tmpY < m_CloudHeight && tmpY >= 0 && tmpX >= 0 && tmpX < m_CloudWidth)
			{
				V_CloudPoints[tmpY][tmpX].push_back(*tmp);
				//MinDepth = min(MinDepth, tmp->Z);
				//MaxDepth = max(MaxDepth, tmp->Z);
			}

		}
	}
	cv::Mat CameraImg(m_CloudHeight, m_CloudWidth, CV_8UC1);
	std::vector<std::pair<size_t, size_t> > NoCloudPoints;
	for (size_t i = 0; i < m_CloudHeight; i++)
	{
		for (size_t j = 0; j < m_CloudWidth; j++)
		{
			float sum = 0;
			float size = V_CloudPoints[m_CloudHeight - 1 - i][j].size();
			for (size_t k = 0; k < size; k++)
				sum += V_CloudPoints[m_CloudHeight - 1 - i][j][k].Z;
			if (size > 0)
			{		//Map the Z range of CameraSpace pos to 0~254 range
				//CameraImg.at<UINT8>(i, j) = UINT8((sum / size - MinDepth) / (MaxDepth - MinDepth) * 255);
				CameraImg.at<UINT8>(i, j) = UINT8((sum / size - CLOUDDEPTHBIAS) * CLOUDDEPTHMUL * 1000);
			}
			else
			{		//If have no points , save it and wait to deal later.
				CameraImg.at<UINT8>(i, j) = 0;
				NoCloudPoints.push_back(std::make_pair(i, j));
			}
		}
	}

	for (size_t i = 0; i < NoCloudPoints.size(); i++)
	{		//find the poses if it is Len distance far from the NoCLoudPoints[i] pos, and set the average of them as Z of the empty pos.
			//If have no neighbor in Len distance ,Len++;
		unsigned int Len = 1;
		unsigned int Count = 0;
		unsigned int Sum = 0;
		size_t x, y;
		while (Len < m_CloudHeight&&Len < m_CloudWidth)
		{
			x = NoCloudPoints[i].first - Len;
			y = NoCloudPoints[i].second - Len;
			if (x >= 0 && x < m_CloudHeight&&y >= 0 && y < m_CloudWidth)
			{
				if (CameraImg.at<UINT8>(x, y) != 0)
				{
					Sum += CameraImg.at<UINT8>(x, y);
					Count++;
				}
			}
			x = NoCloudPoints[i].first - Len;
			y = NoCloudPoints[i].second;
			if (x >= 0 && x < m_CloudHeight&&y >= 0 && y < m_CloudWidth)
			{
				if (CameraImg.at<UINT8>(x, y) != 0)
				{
					Sum += CameraImg.at<UINT8>(x, y);
					Count++;
				}
			}
			x = NoCloudPoints[i].first - Len;
			y = NoCloudPoints[i].second + Len;
			if (x >= 0 && x < m_CloudHeight&&y >= 0 && y < m_CloudWidth)
			{
				if (CameraImg.at<UINT8>(x, y) != 0)
				{
					Sum += CameraImg.at<UINT8>(x, y);
					Count++;
				}
			}
			x = NoCloudPoints[i].first ;
			y = NoCloudPoints[i].second - Len;
			if (x >= 0 && x < m_CloudHeight&&y >= 0 && y < m_CloudWidth)
			{
				if (CameraImg.at<UINT8>(x, y) != 0)
				{
					Sum += CameraImg.at<UINT8>(x, y);
					Count++;
				}
			}
			x = NoCloudPoints[i].first ;
			y = NoCloudPoints[i].second + Len;
			if (x >= 0 && x < m_CloudHeight&&y >= 0 && y < m_CloudWidth)
			{
				if (CameraImg.at<UINT8>(x, y) != 0)
				{
					Sum += CameraImg.at<UINT8>(x, y);
					Count++;
				}
			}
			x = NoCloudPoints[i].first + Len;
			y = NoCloudPoints[i].second - Len;
			if (x >= 0 && x < m_CloudHeight&&y >= 0 && y < m_CloudWidth)
			{
				if (CameraImg.at<UINT8>(x, y) != 0)
				{
					Sum += CameraImg.at<UINT8>(x, y);
					Count++;
				}
			}
			x = NoCloudPoints[i].first + Len;
			y = NoCloudPoints[i].second;
			if (x >= 0 && x < m_CloudHeight&&y >= 0 && y < m_CloudWidth)
			{
				if (CameraImg.at<UINT8>(x, y) != 0)
				{
					Sum += CameraImg.at<UINT8>(x, y);
					Count++;
				}
			}
			x = NoCloudPoints[i].first + Len;
			y = NoCloudPoints[i].second + Len;
			if (x >= 0 && x < m_CloudHeight&&y >= 0 && y < m_CloudWidth)
			{
				if (CameraImg.at<UINT8>(x, y) != 0)
				{
					Sum += CameraImg.at<UINT8>(x, y);
					Count++;
				}
			}
			if (Count != 0)
			{
				CameraImg.at<UINT8>(NoCloudPoints[i].first, NoCloudPoints[i].second) = (UINT8)(Sum / Count);
				break;
			}
			Len++;
		}
	}
	//Notice we can change the size of cv::Mat in subprogram,so we dont set size and type of CloudPointsImg in MainProgram.
	CameraImg.copyTo(*Graph->CloudPointsImg);
	delete[]ColorImg;
	return res;
}

GT_RES	KinectDriver::DepthConvertMat(const UINT16* pBuffer, const unsigned int nWidth, const unsigned int nHeight, cv::Mat *pImg)
{
	for (size_t i = 0; i < nHeight; i++)
	{
		for (size_t j = 0; j < nWidth; j++)
		{
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

GT_RES	KinectDriver::ColorDepth2Robot(const GraspPose posColor, Pose3D &posUR)
{
	GT_RES res;
	ColorSpacePoint Colorpos;
	CameraSpacePoint Camerapos;
	Colorpos.X = posColor.x;
	Colorpos.Y = posColor.y;
	//是否要取多个点做平均？？
	res = Colorpos2Camerapos(Colorpos, Camerapos);
	if (res != GT_RES_OK)
	{
		return res;
	}
	//transform the CameraSpace pos to RobotSpace pos by Matrix T,Mat_Robot = Mat_Trans * Mat_Camera(Mat_Trans is got by calibrated in CoordinateCaliabration project)
	cv::Mat MatCamera(cv::Size(1, 4), CV_32F);
	cv::Mat MatUR(cv::Size(1, 3), CV_32F);
	cv::Mat MatTrans(cv::Size(4, 3), CV_32F);
	MatCamera.at<float>(0, 0) = Camerapos.X;
	MatCamera.at<float>(1, 0) = Camerapos.Y;
	MatCamera.at<float>(2, 0) = Camerapos.Z;
	MatCamera.at<float>(3, 0) = 1.0;
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
	MatUR = MatTrans*MatCamera;																	//Robot=Trans*Camera

	posUR.x = MatUR.at<float>(0, 0);
	posUR.y = MatUR.at<float>(1, 0);
	posUR.z = MatUR.at<float>(2, 0)+0.005;
	posUR.Rx = PI;																				//Default Robot TCP is towards down;
	posUR.Ry = 0;
	posUR.Rz = posColor.theta + ANGLEBIAS;														//Anglebias is calibrated or learn by nerual network.
	return GT_RES_OK;
}

GT_RES	KinectDriver::Colorpos2Cloudpos(const unsigned int ColorposX, const unsigned int ColorposY, unsigned int &Cloudx, unsigned int &Cloudy)
{
	CameraSpacePoint Camerapos;
	ColorSpacePoint Colorpos;
	Colorpos.X = ColorposX;
	Colorpos.Y = ColorposY;
	GT_RES res = Colorpos2Camerapos(Colorpos, Camerapos);
	if (res != GT_RES_OK)
	{
		return res;
	}
	Cloudx = (unsigned int)((Camerapos.X * 1000 - m_CloudWidthBias) / CLOUDRESOLUTION + 0.5);
	Cloudy = m_CloudHeight - 1 - (unsigned int)((Camerapos.Y * 1000 - m_CloudHeightBias) / CLOUDRESOLUTION + 0.5);
	return res;
}

GT_RES	KinectDriver::Colorpos2Camerapos(const ColorSpacePoint Colorpos, CameraSpacePoint &Camerapos)
{
	GT_RES res;
	HRESULT hr;
	//Check the kinect is ready
	if (!m_pKinectSensor)
	{
		printf("Kinect have not initilized.\n");
		return GT_RES_DEVICENOTREADY;
	}
	BOOLEAN Open = false;
	m_pKinectSensor->get_IsOpen(&Open);
	if (!Open)
	{
		printf("KinectSensor have not opened.\n");
		return GT_RES_DEVICENOTREADY;
	}
	if (!m_pCoordinateMapper)
	{
		printf("CoordinateMapper have not been ready.\n");
		return GT_RES_DEVICENOTREADY;
	}
	//Check if DepthImage have been got, if not , get it!(this maxtrix will be delete when get Kinect image)
	if (!m_pDepthImage)
	{
		m_pDepthImage = new UINT16[DEPTHWIDTH*DEPTHHEIGHT];
		res = GetDepthImage(m_pDepthImage);
		if (res != GT_RES_OK)
		{
			printf("GetDepthImage failed!\n");
			delete[]m_pDepthImage;
			m_pDepthImage = NULL;
			return res;
		}
	}
	//Check if Colorframe to Camera Space Matrix have been got, if not , get it!(this maxtrix will be delete when get Kinect image)
	if (!m_pColorInCameraSpace)
	{
		m_pColorInCameraSpace = new CameraSpacePoint[COLORHEIGHT*COLORWIDTH];
		hr = m_pCoordinateMapper->MapColorFrameToCameraSpace(DEPTHWIDTH*DEPTHHEIGHT, m_pDepthImage, COLORWIDTH*COLORHEIGHT, m_pColorInCameraSpace);
		if (FAILED(hr))
		{
			printf("MapColorFrameToCameraSpace failed!\n");
			return GT_RES_ERROR;
		}
	}
	//Get CameraSpace pos from Color pos,(Xrgb,Yrgb,Depth)->(Xc,Yc,Zc)
	Camerapos = m_pColorInCameraSpace[((unsigned int)Colorpos.Y*COLORWIDTH + (unsigned int)Colorpos.X)];
	return GT_RES_OK;
}

GT_RES	KinectDriver::Colorpos2Camerapos(const std::vector<ColorSpacePoint> Colorpos, std::vector<CameraSpacePoint> &Camerapos)
{
	size_t posSize = Colorpos.size();
	for (size_t i = 0; i < posSize; i++)
	{
		CameraSpacePoint pos_tmp;
		GT_RES res;
		res = Colorpos2Camerapos(Colorpos[i], pos_tmp);
		if (res != GT_RES_OK)
		{
			printf("Colorpos to Camerapos failed with error:%02x\n", res);
			return res;
		}
		Camerapos.push_back(pos_tmp);
	}
	return GT_RES_OK;
}

GT_RES	KinectDriver::Colorpos2Depthpos(const ColorSpacePoint Colorpos, DepthSpacePoint &Depthpos)
{
	GT_RES res;
	HRESULT hr;
	//Check the kinect is ready
	if (!m_pKinectSensor)
	{
		printf("Kinect have not initilized.\n");
		return GT_RES_DEVICENOTREADY;
	}
	BOOLEAN Open = false;
	m_pKinectSensor->get_IsOpen(&Open);
	if (!Open)
	{
		printf("KinectSensor have not opened.\n");
		return GT_RES_DEVICENOTREADY;
	}
	if (!m_pCoordinateMapper)
	{
		printf("CoordinateMapper have not been ready.\n");
		return GT_RES_DEVICENOTREADY;
	}
	//Check if DepthImage have been got, if not , get it!(this maxtrix will be delete when get Kinect image)
	if (!m_pDepthImage)
	{
		m_pDepthImage = new UINT16[DEPTHWIDTH*DEPTHHEIGHT];
		res = GetDepthImage(m_pDepthImage);
		if (res != GT_RES_OK)
		{
			printf("GetDepthImage failed!\n");
			delete[]m_pDepthImage;
			m_pDepthImage = NULL;
			return res;
		}
	}
	//Check if Colorframe to Depth Space Matrix have been got, if not , get it!(this maxtrix will be delete when get Kinect image)
	if (!m_pColorInDepthSpace)
	{
		m_pColorInDepthSpace = new DepthSpacePoint[COLORWIDTH*COLORHEIGHT];
		hr = m_pCoordinateMapper->MapColorFrameToDepthSpace(DEPTHWIDTH*DEPTHHEIGHT, m_pDepthImage, COLORWIDTH*COLORHEIGHT, m_pColorInDepthSpace);
		if (FAILED(hr))
		{
			printf("MapColorFrameToDepthSpace failed!\n");
			return GT_RES_ERROR;
		}
	}
	//Get DepthSpace pos from Color pos,(Xrgb,Yrgb)->Depth
	Depthpos = m_pColorInDepthSpace[((unsigned int)Colorpos.Y*COLORWIDTH + (unsigned int)Colorpos.X)];
	return GT_RES_OK;
}

GT_RES	KinectDriver::Colorpos2Depthpos(const std::vector<ColorSpacePoint> Colorpos, std::vector<DepthSpacePoint> &Depthpos)
{
	size_t posSize = Colorpos.size();
	for (size_t i = 0; i < posSize; i++)
	{
		DepthSpacePoint pos_tmp;
		GT_RES res;
		res = Colorpos2Depthpos(Colorpos[i], pos_tmp);
		if (res != GT_RES_OK)
		{
			printf("Colorpos to Depthpos failed with error:%02x\n", res);
			return res;
		}
		Depthpos.push_back(pos_tmp);
	}
	return GT_RES_OK;
}

GT_RES	KinectDriver::Colorpos2Depthpos(const float Xc, const float Yc, float &Xd, float &Yd)
{
	GT_RES res;
	ColorSpacePoint Colorpos;
	DepthSpacePoint Depthpos;
	Colorpos.X = Xc;
	Colorpos.Y = Yc;
	res = Colorpos2Depthpos(Colorpos, Depthpos);
	if (res != GT_RES_OK)
	{
		return res;
	}
	Xd = Depthpos.X;
	Yd = Depthpos.Y;
	return GT_RES_OK;
}
