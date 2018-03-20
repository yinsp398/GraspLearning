
#include "NN.h"
#include "KinectDriver.h"
#include <string>
#include <stdio.h>
#include <iostream>

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
	GraphIn->ColorImg = new cv::Mat(COLORWIDTH, COLORHEIGHT, COLORFORMAT);
	GraphIn->DepthImg = new cv::Mat(DEPTHWIDTH, DEPTHHEIGHT, COLORFORMAT);
	GraphOut = new Graphics;
	GraphOut->ColorImg = new cv::Mat(COLORGRAPHWIDTH, COLORGRAPHHEIGHT, COLORFORMAT);
	GraphOut->DepthImg = new cv::Mat(DEPTHGRAPHWIDTH, DEPTHGRAPHHEIGHT, COLORFORMAT);
	cv::Mat * pMat = new cv::Mat(DEPTHWIDTH, DEPTHHEIGHT, DEPTHFORMAT);
	while (1)
	{
		std::cin.get();

		cv::Mat * tmp;
		tmp = GraphIn->DepthImg;
		GraphIn->DepthImg = pMat;
		pMat = tmp;

		res = Kinect->GetKinectImage(GraphIn);
		GetBYTEformat(GraphIn->DepthImg, pMat);

		tmp = GraphIn->DepthImg;
		GraphIn->DepthImg = pMat;
		pMat = tmp;

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
				pos.theta = rand() / double(RAND_MAX) * PI;
				for (size_t k = 0; k < 1; k++)
				{											//是否翻转
					for (size_t p = 0; p < 4; p++)
					{										//不同的对比度
						double contrast = rand() / double(RAND_MAX)*0.6 + 0.7;
						if ((res = NNet->GetSubImage(&pos, GraphIn, GraphOut, k, true, contrast)) != GT_RES_OK)
						{
							printf("Get subimage failed!\n");
							continue;
						}
						cv::imwrite("..\\Image\\Color" + Int2Str(ImgCnt, 5) + ".jpg", *(GraphOut->ColorImg));
						cv::imwrite("..\\Image\\Depth" + Int2Str(ImgCnt, 5) + ".jpg", *(GraphOut->DepthImg));
					}
				}
			}
		}
	}

	return 0;
}