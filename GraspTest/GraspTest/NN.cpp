
#include "NN.h"
#include "Classification.h"
#include "utils.h"
#ifndef _CameraSpacePoint_
#define _CameraSpacePoint_
typedef struct _CameraSpacePoint
{
	float X;
	float Y;
	float Z;
} 	CameraSpacePoint;

#endif // _CameraSpacePoint_
NN::NN(	const std::string & model_file,
		const std::string & trained_file,
		const std::string & mean_file1,
		const std::string & mean_file2)
{
	m_pClassifier = new Classifier(model_file, trained_file, mean_file1, mean_file2);
	m_ppos = new std::pair<GraspPose, float>;
	m_ppos->second = 0;
	m_pposes = new std::vector<std::pair<GraspPose, float> >;
	m_pColorInCameraSpace = new CameraSpacePoint[COLORWIDTH*COLORHEIGHT];
	m_pUpdateTime = new time_t;
	*m_pUpdateTime = time(NULL);

	srand((unsigned)time(NULL));											//initialize the rand seed by time stamp
	m_pGraph = new Graphics;
	//m_pGraph->DepthImg = new cv::Mat(DEPTHHEIGHT, DEPTHWIDTH, COLORFORMAT);
	m_pGraph->ColorImg = new cv::Mat(COLORHEIGHT, COLORWIDTH, COLORFORMAT);
	m_pGraph->CloudPointsImg = new cv::Mat;
}

NN::~NN()
{
	delete m_pCoordinateMap;
	delete m_pClassifier;
	delete m_ppos;
	delete m_pposes;
	delete m_pUpdateTime;
	//delete m_pGraph->DepthImg;
	delete m_pGraph->ColorImg;
	delete m_pGraph;

}

bool NN::IsInLimits(GraspPose *pos)
{
	float len = COLORGRAPHHEIGHT / 3.0;
	if (pos->x + fabs(len*cos(pos->theta)) > COLORSPACERIGHT)
	{
		return false;
	}
	else if (pos->x - fabs(len*cos(pos->theta)) < COLORSPACELEFT)
	{
		return false;
	}
	else if (pos->y + fabs(len*sin(pos->theta)) > COLORSPACEDOWN)
	{
		return false;
	}
	else if (pos->y - fabs(len*sin(pos->theta)) < COLORSPACEUP)
	{
		return false;
	}
	else if (pos->theta<0 || pos->theta > PI)
	{
		return false;
	}
	unsigned int tmpx = pos->x;
	unsigned int tmpy = pos->y;
	unsigned int left = m_ppos->first.x - COLORGRAPHWIDTH / 2;
	unsigned int right = m_ppos->first.x + COLORGRAPHWIDTH / 2;
	unsigned int up = m_ppos->first.y - COLORGRAPHHEIGHT / 2;
	unsigned int down = m_ppos->first.y + COLORGRAPHHEIGHT / 2;	
	//make sure not last pos around
	if ((tmpx < right) && (tmpx > left) && (tmpy < down) && (tmpy > up))
	{
		return false;
	}
	return true;
}

float GaussRand(float mu, float sigma)
{
	const double epsilon = std::numeric_limits<double>::min();
	const double two_pi = 2.0*3.14159265358979323846;

	static double z0, z1;
	static bool generate;
	generate = !generate;

	if (!generate)
		return z1 * sigma + mu;
	double u1, u2;
	do
	{
		u1 = rand() * (1.0 / RAND_MAX);
		u2 = rand() * (1.0 / RAND_MAX);
	} while (u1 <= epsilon);

	z0 = sqrt(-2.0 * log(u1)) * cos(two_pi * u2);
	z1 = sqrt(-2.0 * log(u1)) * sin(two_pi * u2);
	return z0 * sigma + mu;
}
//update Net parameter
GT_RES	NN::UpdateParam(std::string trained_file, std::string mean_file1, std::string mean_file2)
{
	m_pClassifier->Updatefile(mean_file1, mean_file2, trained_file);
	return GT_RES_OK;
}

//Update graphics
GT_RES	NN::UpdateGraphics(Graphics * graph, CameraSpacePoint* ColorInCameraSpace, int CloudWidthBias, int CloudHeightBias)
{	
	if (graph == NULL)
	{
		printf("Have no graphics.\n");
		return GT_RES_ERROR;
	}
	graph->ColorImg->copyTo(*(m_pGraph->ColorImg));
	graph->CloudPointsImg->copyTo(*(m_pGraph->CloudPointsImg));
	memcpy(m_pColorInCameraSpace, ColorInCameraSpace, COLORWIDTH*COLORHEIGHT * sizeof(CameraSpacePoint));
	m_CloudHeightBias = CloudHeightBias;
	m_CloudWidthBias = CloudWidthBias;
	//memcpy(m_pGraph->DepthImg, graph->DepthImg, sizeof(graph->DepthImg));
	//convert depth image from 16bit to 8bit , 0~255 
	/*
	int nWidth = graph->DepthImg->cols;
	int nHeight = graph->DepthImg->rows;
	for (size_t i = 0; i < nHeight; i++)
	{
		for (size_t j = 0; j < nWidth; j++)
		{
			m_pGraph->DepthImg->at<uchar>(i, j) = (uchar)(graph->DepthImg->at<UINT16>(i, j) % 256);
		}
	}
	*/
	return GT_RES_OK;
}

GT_RES	NN::PosVecPush(std::pair<GraspPose, float> PosPair)
{
	if (m_pposes->size() == 0)
	{
		m_pposes->push_back(PosPair);
		return GT_RES_OK;
	}
	for (int i = 0; i < m_pposes->size(); i++)
	{
		if (((*m_pposes)[i]).second < PosPair.second)
		{
			m_pposes->insert(m_pposes->begin() + i, PosPair);
			if (m_pposes->size() > 10)
			{
				m_pposes->pop_back();
			}
			return GT_RES_OK;
		}
	}
	if (m_pposes->size() < 10)
	{
		m_pposes->insert(m_pposes->end(), PosPair);
	}
	return GT_RES_OK;
}

//NN forward and calculate the pose pair
GT_RES	NN::NNRun()
{
	GT_RES res;
	//check if it is time to update param,  update once every hour
	if (*m_pUpdateTime - TIME_UPDATEPARAM > time(NULL))
	{
		UpdateParam(TRAINEDFILE, MEANFILE1, MEANFILE2);
		*m_pUpdateTime = time(NULL);
	}
	if (m_pGraph == NULL)
	{
		printf("The pointer to image is NULL.\n");
		return GT_RES_ERROR;
	}
	//这里先采用最简单直接的方式，直接暴力找，之后调通了再修改优化这里
	//直接使用彩色图像，之后再考虑深度图像
	Graphics * Subgraph = new Graphics;
	Subgraph->ColorImg = new cv::Mat(COLORGRAPHHEIGHT,COLORGRAPHWIDTH,COLORFORMAT);
	Subgraph->CloudPointsImg = new cv::Mat(CLOUDGRAPHHEIGHT, CLOUDGRAPHWIDTH, COLORFORMAT);
	std::vector<GraspPose> posV;
	if ((res = GetRandomPoses(&posV)) != GT_RES_OK)
	{
		printf("Get random pose failed with error: %02x.\n", res);
		delete Subgraph->ColorImg;
		delete Subgraph->CloudPointsImg;
		delete Subgraph;
		return res;
	}
	for (int i = 0; i < 50; i++)
	{
		if ((res = GetSubImage(&posV[i], m_pGraph, Subgraph,false,false,1)) != GT_RES_OK)
		{
			printf("Get Subimage failed with error: %02x.\n", res);
			delete Subgraph->ColorImg;
			delete Subgraph->CloudPointsImg;
			delete Subgraph;
			return res;
		}
		std::vector<float> Vec_Tmp = m_pClassifier->Predict(*(Subgraph->ColorImg), *(Subgraph->CloudPointsImg));

		std::pair<GraspPose, float> Pair_tmp = std::make_pair(posV[i], Vec_Tmp[1]);
		PosVecPush(Pair_tmp);
	}
	/*
	printf("PosVec:\n");
	for (int i = 0; i < m_pposes->size(); i++)
	{
		printf("%d  %6f,\t", i, ((*m_pposes)[i]).second);
	}
	printf("\n");*/
	delete Subgraph->ColorImg;
	delete Subgraph->CloudPointsImg;
	delete Subgraph;
	return GT_RES_OK;
}


//Get random pose one by one (algorithm 1)
GT_RES	NN::GetRandomPose(GraspPose * pos)
{
	
	do {
		pos->x = rand() % (COLORSPACERIGHT - COLORSPACELEFT) + COLORSPACELEFT;
		pos->y = rand() % (COLORSPACEDOWN - COLORSPACEUP) + COLORSPACEUP;
		pos->theta = rand() / double(RAND_MAX) * PI;
	} while (!IsInLimits(pos));

	return GT_RES_OK;
}

//Get random pose by group (algorithm 2)
GT_RES	NN::GetRandomPoses(std::vector<GraspPose> *vec_pos)
{
	GraspPose tmp;
	for (int i = 0; i < m_pposes->size(); i++)
	{
		for (int j = 0; j < 4; j++)
		{
			do {
				tmp.x = (unsigned int)((*m_pposes)[i].first.x + GaussRand(0, 4));
				tmp.y = (unsigned int)((*m_pposes)[i].first.y + GaussRand(0, 4));
				tmp.theta = (*m_pposes)[i].first.theta + GaussRand(0, 0.5);
			} while (!IsInLimits(&tmp));
			vec_pos->push_back(tmp);
		}
	}
	for (int i = 0;50 > vec_pos->size(); i++)
	{
		do {
			tmp.x = rand() % (COLORSPACERIGHT - COLORSPACELEFT) + COLORSPACELEFT;
			tmp.y = rand() % (COLORSPACEDOWN - COLORSPACEUP) + COLORSPACEUP;
			tmp.theta = rand() / double(RAND_MAX) * PI;
		} while (!IsInLimits(&tmp));
		(vec_pos)->push_back(tmp);
	}
	return GT_RES_OK;
}

GT_RES	NN::PosVecClear(std::pair<GraspPose, float> *PosPair)
{
	unsigned int left = PosPair->first.x - COLORGRAPHWIDTH / 2;
	unsigned int right = PosPair->first.x + COLORGRAPHWIDTH / 2;
	unsigned int up = PosPair->first.y - COLORGRAPHHEIGHT / 2;
	unsigned int down = PosPair->first.y + COLORGRAPHHEIGHT / 2;

	for (int i = 0; i < m_pposes->size(); i++)
	{
		unsigned int tmpx = ((*m_pposes)[i]).first.x;
		unsigned int tmpy = ((*m_pposes)[i]).first.y;
		if ((tmpx < right) && (tmpx > left) && (tmpy < down) && (tmpy > up))
		{
			m_pposes->erase(m_pposes->begin() + i, m_pposes->begin() + i + 1);
			i--;
		}
	}
	return GT_RES_OK;
}
//get the optimal Pose
GT_RES	NN::GetPose(Pose3D *pos, const unsigned int ImgCnt)
{
	GT_RES res;
	//debug test
	//GetRandomPose(&(m_ppos->first));
	//m_ppos->second = 0;
	//debug test

	if (m_ppos)
	{
		*m_ppos = m_pposes->back();
		m_pposes->pop_back();
		printf("Prediction:%6f\%\n", m_ppos->second*100.0);
		Graphics * Subgraph = new Graphics;
		Subgraph->ColorImg = new cv::Mat(COLORGRAPHHEIGHT, COLORGRAPHWIDTH, COLORFORMAT);
		Subgraph->CloudPointsImg = new cv::Mat(CLOUDGRAPHHEIGHT, CLOUDGRAPHWIDTH, COLORFORMAT);
		//save the subimage about the m_ppos to file
		if ((res = GetSubImage(&(m_ppos->first), m_pGraph, Subgraph, false, false, 1)) != GT_RES_OK)
		{
			printf("Get Subimage failed with error: %02x.\n", res);
			delete Subgraph->ColorImg;
			delete Subgraph->CloudPointsImg;
			delete Subgraph;
			return res;
		}
		string FileNameColor,FileNameCloudPoints;
		FileNameColor = IMAGEPATH;
		FileNameCloudPoints = CLOUDPATH;
		std::stringstream ss;
		string str_tmp;
		ss << ImgCnt;
		ss >> str_tmp;
		if (5 > str_tmp.size())
		{
			str_tmp.insert(0, 5 - str_tmp.size(), '0');
		}
		FileNameColor += str_tmp;
		FileNameColor += ".jpg";
		FileNameCloudPoints += str_tmp;
		FileNameCloudPoints += ".jpg";
		cv::imwrite(FileNameColor, *(Subgraph->ColorImg));
		cv::imwrite(FileNameCloudPoints, *(Subgraph->CloudPointsImg));
		
		//save the predicted possibility to file
		std::ofstream out;
		out.open(PREDICTPATH, std::ios::app);
		out << str_tmp << ".jpg " << m_ppos->second << std::endl;
		out.close();

		res = ColorDepth2Robot(m_ppos->first, *pos);
		Pose3D Pos3Dtmp[8];
		GraspPose postmp[8];
		float length1 = COLORGRAPHHEIGHT / 3.0, length2 = COLORGRAPHHEIGHT / 2.0;
		float length3 = (length1 + length2) / 2;
		postmp[0].x = m_ppos->first.x - length1*cos(m_ppos->first.theta);
		postmp[0].y = m_ppos->first.y - length1*sin(m_ppos->first.theta);
		postmp[1].x = m_ppos->first.x - length2*cos(m_ppos->first.theta);
		postmp[1].y = m_ppos->first.y - length2*sin(m_ppos->first.theta);
		postmp[2].x = m_ppos->first.x - length3*cos(m_ppos->first.theta);
		postmp[2].y = m_ppos->first.y - length3*sin(m_ppos->first.theta + 0.0828);
		postmp[3].x = m_ppos->first.x - length3*cos(m_ppos->first.theta);
		postmp[3].y = m_ppos->first.y - length3*sin(m_ppos->first.theta - 0.0828);
		postmp[4].x = m_ppos->first.x + length1*cos(m_ppos->first.theta);
		postmp[4].y = m_ppos->first.y + length1*sin(m_ppos->first.theta);
		postmp[5].x = m_ppos->first.x + length2*cos(m_ppos->first.theta);
		postmp[5].y = m_ppos->first.y + length2*sin(m_ppos->first.theta);
		postmp[6].x = m_ppos->first.x + length3*cos(m_ppos->first.theta);
		postmp[6].y = m_ppos->first.y + length3*sin(m_ppos->first.theta + 0.0828);
		postmp[7].x = m_ppos->first.x + length3*cos(m_ppos->first.theta);
		postmp[7].y = m_ppos->first.y + length3*sin(m_ppos->first.theta - 0.0828);

		pos->z = 0;
		for (int i = 0; i < 8; i++)
		{
			ColorDepth2Robot(postmp[i], Pos3Dtmp[i]);
			if (pos->z < Pos3Dtmp[i].z)
			{
				pos->z = Pos3Dtmp[i].z;
			}
		}
		m_pposes->clear();
		PosVecClear(m_ppos);
		delete Subgraph->ColorImg;
		delete Subgraph->CloudPointsImg;
		delete Subgraph;
		return GT_RES_OK;
	}
	else
	{
		printf("Have not initialized the NN\n");
		return GT_RES_ERROR;
	}
}

GT_RES	NN::ColorDepth2Robot(const GraspPose posColor, Pose3D &posUR)
{
	GT_RES res;

	CameraSpacePoint CameraPos;
	CameraPos = m_pColorInCameraSpace[(posColor.y*COLORWIDTH + posColor.x)];

	//transform the CameraSpace pos to RobotSpace pos by Matrix T,Mat_Robot = Mat_Trans * Mat_Camera(Mat_Trans is got by calibrated in CoordinateCaliabration project)
	cv::Mat MatCamera(cv::Size(1, 4), CV_32F);
	cv::Mat MatUR(cv::Size(1, 3), CV_32F);
	cv::Mat MatTrans(cv::Size(4, 3), CV_32F);
	MatCamera.at<float>(0, 0) = CameraPos.X;
	MatCamera.at<float>(1, 0) = CameraPos.Y;
	MatCamera.at<float>(2, 0) = CameraPos.Z;
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
	posUR.z = MatUR.at<float>(2, 0) + 0.005;
	posUR.Rx = PI;																				//Default Robot TCP is towards down;
	posUR.Ry = 0;
	posUR.Rz = posColor.theta + ANGLEBIAS;														//Anglebias is calibrated or learn by nerual network.
	return GT_RES_OK;
}
//Get graph by the center of pose
GT_RES	NN::GetSubImage(GraspPose * const pos, Graphics* const GraphIn, Graphics *GraphOut, const bool flip, const bool contrast, const float contrast_scale)
{
	if (pos == NULL || GraphIn == NULL || GraphOut == NULL)
	{
		printf("The pointer to pos or image is NULL.\n");
		return GT_RES_ERROR;
	}
	//确保二者尺寸一致
	if (GraphIn->ColorImg->rows != COLORHEIGHT || GraphIn->ColorImg->cols != COLORWIDTH)
	{
		printf("The size of GraphIn is not right.\n");
		return GT_RES_ERROR;
	}
	if (GraphOut->ColorImg->rows != COLORGRAPHHEIGHT || GraphOut->ColorImg->cols != COLORGRAPHWIDTH || GraphOut->CloudPointsImg->rows != CLOUDGRAPHHEIGHT || GraphOut->CloudPointsImg->cols != CLOUDGRAPHWIDTH)
	{
		printf("The size of GraphOut is not right.\n");
		return GT_RES_ERROR;
	}
	if (GraphIn->ColorImg->type() != GraphOut->ColorImg->type() || GraphIn->CloudPointsImg->type() != GraphOut->CloudPointsImg->type())
	{
		printf("The type of GraphIn and GraphOut is not consistent.\n");
		return GT_RES_ERROR;
	}
	//边界检查
	if (pos->x > COLORSPACERIGHT || pos->x < COLORSPACELEFT)
	{
		printf("The center position x is out of the boundary.\n");
		return GT_RES_ERROR;
	}
	else if (pos->y > COLORSPACEDOWN || pos->y < COLORSPACEUP)
	{
		printf("The center position y is out of the boundary.\n");
		return GT_RES_ERROR;
	}
	else if (pos->theta<0 || pos->theta>PI)
	{
		printf("The center position theta is out of the boundary.\n");
		return GT_RES_ERROR;
	}
	
	unsigned int Cloudx, Cloudy;
	//Get Pos in Cloudpoints
	//first,Get CameraPos
	CameraSpacePoint CameraPos;
	CameraPos = m_pColorInCameraSpace[(pos->y*COLORWIDTH + pos->x)];
	//convert camerapos to CloudpointsImg
	Cloudx = (unsigned int)(((CameraPos.X * 1000 - m_CloudWidthBias) / CLOUDRESOLUTION + 0.5));
	Cloudy = GraphIn->CloudPointsImg->rows - 1 - (unsigned int)(((CameraPos.Y * 1000 - m_CloudHeightBias) / CLOUDRESOLUTION + 0.5));
	if (Cloudx > 300 || Cloudx < 50 || Cloudy > 300 || Cloudy < 50)
	{
		printf("Cloud pos is out of range:%d,%d\n", Cloudx, Cloudy);
		return GT_RES_ERROR;
	}
	cv::Point2f Center1(pos->x, pos->y);
	cv::Point2f Center2(Cloudx, Cloudy);
	cv::Mat *pMat1 = GraphIn->ColorImg;
	cv::Mat *pMat2 = GraphIn->CloudPointsImg;
	cv::Mat *pMat = NULL;

	if (fabs(pos->theta) > 1e-5)
	{
		pMat = new cv::Mat;
		cv::Mat RotationMat1 = cv::getRotationMatrix2D(Center1, pos->theta / PI * 180, COLORSCALE);
		cv::warpAffine(*pMat1, *pMat, RotationMat1, cv::Size(COLORWIDTH, COLORHEIGHT));
		pMat1 = pMat;

		pMat = new cv::Mat;
		cv::Mat RotationMat2 = cv::getRotationMatrix2D(Center2, pos->theta / PI * 180, COLORSCALE);
		cv::warpAffine(*pMat2, *pMat, RotationMat2, cv::Size(GraphIn->CloudPointsImg->cols, GraphIn->CloudPointsImg->rows));
		pMat2 = pMat;
	}


	pMat = new cv::Mat;
	cv::Rect rect1(pos->x - COLORGRAPHWIDTH / 2.0, pos->y - COLORGRAPHHEIGHT / 2.0, COLORGRAPHWIDTH, COLORGRAPHHEIGHT);
	(*pMat1)(rect1).copyTo(*pMat);
	if (fabs(pos->theta) > 1e-5)
		delete pMat1;
	pMat1 = pMat;

	pMat = new cv::Mat;
	cv::Rect rect2(Cloudx - CLOUDGRAPHWIDTH / 2.0, Cloudy - CLOUDGRAPHHEIGHT / 2.0, CLOUDGRAPHWIDTH, CLOUDGRAPHHEIGHT);
	(*pMat2)(rect2).copyTo(*pMat);
	if (fabs(pos->theta) > 1e-5)
		delete pMat2;
	pMat2 = pMat;



	if (contrast)
	{
		size_t width1 = pMat1->cols;
		size_t height1 = pMat1->rows;
		for (size_t i = 0; i < height1; i++)
		{
			for (size_t j = 0; j < width1; j++)
			{
				pMat1->at<uchar>(i, j) = cv::saturate_cast<uchar>(pMat1->at<uchar>(i, j)*contrast_scale);
			}
		}

		size_t width2 = pMat2->cols;
		size_t height2 = pMat2->rows;
		for (size_t i = 0; i < height2; i++)
		{
			for (size_t j = 0; j < width2; j++)
			{
				pMat2->at<uchar>(i, j) = cv::saturate_cast<uchar>(pMat2->at<uchar>(i, j)*contrast_scale);
			}
		}
	}

	if (flip)
	{
		cv::flip(*pMat1, *(GraphOut->ColorImg), 0);
		cv::flip(*pMat2, *(GraphOut->CloudPointsImg), 0);
	}
	else
	{
		pMat1->copyTo(*(GraphOut->ColorImg));
		pMat2->copyTo(*(GraphOut->CloudPointsImg));
	}
	delete pMat1;
	delete pMat2;
	return GT_RES_OK;
}