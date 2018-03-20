
#include "NN.h"
#include "KinectDriver.h"
#include <string>
#include <stdio.h>
#include <iostream>
#include <windows.h>

bool GetBYTEformat(cv::Mat *DepthImg, cv::Mat *OutImg)
{
	int nWidth = DepthImg->size.p[1];
	int nHeight = DepthImg->size.p[0];
	for (size_t i = 0; i < nHeight; i++)
	{
		for (size_t j = 0; j < nWidth; j++)
		{
			OutImg->at<uchar>(i, j) = (uchar)(DepthImg->at<UINT16>(i, j) % 256);
		}
	}
	return true;
}

std::string Int2Str(const unsigned int Cnt, const unsigned int prefix)
{
	std::string str = std::to_string(Cnt);
	if (prefix > str.size())
	{
		str.insert(0, prefix - str.size(), '0');
	}
	return str;
}

int main()
{
	GT_RES res;
	unsigned int ImgCnt = 0;
	KinectDriver *Kinect;
	Kinect = new KinectDriver;
	std::string str1, str2, str3;
	NN* NNet = new NN("", "", "", Kinect);
	Graphics *GraphIn, *GraphOut;
	GraphIn = new Graphics;
	GraphIn->ColorImg = new cv::Mat(COLORHEIGHT, COLORWIDTH, COLORFORMAT);
	GraphIn->DepthImg = new cv::Mat(DEPTHHEIGHT, DEPTHWIDTH, COLORFORMAT);
	GraphOut = new Graphics;
	GraphOut->ColorImg = new cv::Mat(COLORGRAPHHEIGHT, COLORGRAPHWIDTH, COLORFORMAT);
	GraphOut->DepthImg = new cv::Mat(DEPTHGRAPHHEIGHT, DEPTHGRAPHWIDTH, COLORFORMAT);
	cv::Mat * pMat = new cv::Mat(DEPTHHEIGHT, DEPTHWIDTH, DEPTHFORMAT);
	Sleep(5000);
	while (1)
	{
		if (std::cin.get() == ' ')
			break;
		
		cv::Mat * tmp;
		tmp = GraphIn->DepthImg;
		GraphIn->DepthImg = pMat;
		pMat = tmp;

		res = Kinect->GetKinectImage(GraphIn);
		GetBYTEformat(GraphIn->DepthImg, pMat);

		tmp = GraphIn->DepthImg;
		GraphIn->DepthImg = pMat;
		pMat = tmp;

		cv::imwrite("..\\ImageSet\\ColorImg.jpg", *(GraphIn->ColorImg));
		cv::imwrite("..\\ImageSet\\DepthImg.jpg", *(GraphIn->DepthImg));
		if (res != GT_RES_OK)
		{
			printf("Getting Kinect image failed with error:%02x\n", res);
			return res;
		}
		GraspPose pos ;
		for (size_t i = 0; i < 10; i++)
		{													//随机获取十个不同位置的图片
			if ((res = NNet->GetRandomPose(&pos)) != GT_RES_OK)
			{
				printf("Getrandompos failed!\n");
				continue;
			}
			for (size_t j = 0; j < 10; j++)
			{												//随机获取十个不同角度的图片
				//pos.theta = rand() / double(RAND_MAX) * PI;
				pos.theta = j / 10.0 * PI;
				for (size_t k = 0; k < 2; k++)
				{											//是否翻转
					for (size_t p = 0; p < 5; p++)
					{										//不同的对比度
						//double contrast = rand() / double(RAND_MAX)*0.6 + 0.7;
						double contrast = 0.7 + p / 4.0 * 0.6;
						if ((res = NNet->GetSubImage(&pos, GraphIn, GraphOut, k, true, contrast)) != GT_RES_OK)
						{
							printf("Get subimage failed!\n");
							continue;
						}
						cv::imwrite("..\\ImageSet\\Color\\" + Int2Str(ImgCnt, 5) + ".jpg", *(GraphOut->ColorImg));
						std::cout << "..\\ImageSet\\Color\\" + Int2Str(ImgCnt, 5) + ".jpg" << std::endl;
						cv::imwrite("..\\ImageSet\\Depth\\" + Int2Str(ImgCnt, 5) + ".jpg", *(GraphOut->DepthImg));
						std::cout << "..\\ImageSet\\Depth\\" + Int2Str(ImgCnt, 5) + ".jpg" << std::endl;
						ImgCnt++;
					}
				}
			}
		}

		cv::imwrite("..\\ImageSet\\ColorImg.jpg", *(GraphIn->ColorImg));
		cv::imwrite("..\\ImageSet\\DepthImg.jpg", *(GraphIn->DepthImg));

	}

	return 0;
}