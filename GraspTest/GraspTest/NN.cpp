
#include "NN.h"
#include "Classification.h"
#include "KinectDriver.h"
#include "utils.h"

NN::NN(	const std::string & model_file,
		const std::string & trained_file,
		const std::string & mean_file,
		KinectDriver * Kinect)
{
	if (Kinect == NULL)
	{
		std::cout << "have no Kinect Sensor" << std::endl;
	}
	m_pKinect = Kinect;
	//m_pClassifier = new Classifier(model_file, trained_file, mean_file );
	m_ppos = new std::pair<GraspPose, float>;
	m_pposes = new std::vector<std::pair<GraspPose, float> >;
	
	m_pUpdateTime = new time_t;
	*m_pUpdateTime = time(NULL);
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

//update Net parameter
GT_RES	NN::UpdateParam(std::string trained_file, std::string mean_file)
{
	m_pClassifier->Updatefile(mean_file, trained_file);
	return GT_RES_OK;
}

//Update graphics
GT_RES	NN::UpdateGraphics(Graphics * graph)
{	
	if (graph == NULL)
	{
		printf("Have no graphics.\n");
		return GT_RES_ERROR;
	}
	graph->ColorImg->copyTo(*(m_pGraph->ColorImg));
	graph->CloudPointsImg->copyTo(*(m_pGraph->CloudPointsImg));
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

//NN forward and calculate the pose pair
GT_RES	NN::NNRun()
{
	GT_RES res;
	//check if it is time to update param,  update once every hour
	if (*m_pUpdateTime - 3600 > time(NULL))
	{
		UpdateParam(TRAINEDFILE, MEANFILE);
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
	GraspPose pos;
	for (int i = 0; i < 50; i++)
	{
		if ((res = GetRandomPose(&pos)) != GT_RES_OK)
		{
			printf("Get random pose failed with error: %02x.\n", res);
			delete Subgraph->ColorImg;
			delete Subgraph->CloudPointsImg;
			delete Subgraph;
			return res;
		}
		if ((res = GetSubImage(&pos, m_pGraph, Subgraph,false,false,1)) != GT_RES_OK)
		{
			printf("Get Subimage failed with error: %02x.\n", res);
			delete Subgraph->ColorImg;
			delete Subgraph->CloudPointsImg;
			delete Subgraph;
			return res;
		}
		std::vector<float> Vec_Tmp = m_pClassifier->Predict(*(Subgraph->ColorImg), *(Subgraph->CloudPointsImg));
		std::pair<GraspPose, float> Pair_tmp = std::make_pair(pos, Vec_Tmp[0]);
		m_pposes->push_back(Pair_tmp);
		if (m_ppos->second < Vec_Tmp[0])
		{
			m_ppos->second = Vec_Tmp[0];
			m_ppos->first = pos;
		}
	}
	delete Subgraph->ColorImg;
	delete Subgraph->CloudPointsImg;
	delete Subgraph;
	return GT_RES_OK;
}

//Get random pose one by one (algorithm 1)
GT_RES	NN::GetRandomPose(GraspPose * pos)
{
	srand((unsigned)time(NULL));											//initialize the rand seed by time stamp
	pos->x = rand() % (COLORSPACERIGHT - COLORSPACELEFT) + COLORSPACELEFT;
	pos->y = rand() % (COLORSPACEDOWN - COLORSPACEUP) + COLORSPACEUP;
	pos->theta = rand() / double(RAND_MAX) * PI;
	return GT_RES_OK;
}

//Get random pose by group (algorithm 2)
GT_RES	NN::GetRandomPoses(std::vector<GraspPose> *vec_pos)
{
	for (int i = 0; i < 100; i++)
	{
		GraspPose tmp;
		tmp.x = rand() % (COLORSPACERIGHT - COLORSPACELEFT) + COLORSPACELEFT;
		tmp.y = rand() % (COLORSPACEDOWN - COLORSPACEUP) + COLORSPACEUP;
		tmp.theta = rand() / double(RAND_MAX) * PI;
		(vec_pos)->push_back(tmp);
	}
	return GT_RES_OK;
}

//get the optimal Pose
GT_RES	NN::GetPose(Pose3D *pos, const unsigned int ImgCnt)
{
	GT_RES res;
	//debug test
	GetRandomPose(&(m_ppos->first));
	m_ppos->second = 0;
	//debug test

	if (m_ppos)
	{
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
		out << str_tmp << " " << m_ppos->second << std::endl;
		out.close();

		res = m_pKinect->ColorDepth2Robot(m_ppos->first, *pos);
		m_pposes->clear();

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
	GT_RES res = m_pKinect->Colorpos2Cloudpos(pos->x, pos->y, Cloudx, Cloudy);
	if (res != GT_RES_OK)
	{
		printf("Colorpos convert to Cloudpos failed with error:%02x.\n", res);
		return res;
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