
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
	m_pGraph->DepthImg = new cv::Mat(DEPTHHEIGHT, DEPTHWIDTH, COLORFORMAT);
	m_pGraph->ColorImg = new cv::Mat(COLORHEIGHT, COLORWIDTH, COLORFORMAT);

}

NN::~NN()
{
	delete m_pCoordinateMap;
	delete m_pClassifier;
	delete m_ppos;
	delete m_pposes;
	delete m_pUpdateTime;
	delete m_pGraph->DepthImg;
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
	//memcpy(m_pGraph->DepthImg, graph->DepthImg, sizeof(graph->DepthImg));
	//convert depth image from 16bit to 8bit , 0~255 
	int nWidth = graph->DepthImg->size.p[1];
	int nHeight = graph->DepthImg->size.p[0];
	for (size_t i = 0; i < nHeight; i++)
	{
		for (size_t j = 0; j < nWidth; j++)
		{
			m_pGraph->DepthImg->at<uchar>(i, j) = (uchar)(graph->DepthImg->at<UINT16>(i, j) % 256);
		}
	}
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
	//�����Ȳ������ֱ�ӵķ�ʽ��ֱ�ӱ����ң�֮���ͨ�����޸��Ż�����
	//ֱ��ʹ�ò�ɫͼ��֮���ٿ������ͼ��
	Graphics * graph = new Graphics;
	graph->ColorImg = new cv::Mat(COLORHEIGHT,COLORWIDTH,COLORFORMAT);
	graph->DepthImg = new cv::Mat(DEPTHHEIGHT, DEPTHWIDTH, COLORFORMAT);
	GraspPose pos;
	for (int i = 0; i < 50; i++)
	{
		if ((res = GetRandomPose(&pos)) != GT_RES_OK)
		{
			printf("Get random pose failed with error: %02x.\n", res);
			delete graph->ColorImg;
			delete graph->DepthImg;
			delete graph;
			return res;
		}
		if ((res = GetSubImage(&pos, m_pGraph, graph,false,false,1)) != GT_RES_OK)
		{
			printf("Get Subimage failed with error: %02x.\n", res);
			delete graph->ColorImg;
			delete graph->DepthImg;
			delete graph;
			return res;
		}
		std::vector<float> Vec_Tmp = m_pClassifier->Predict(*(graph->ColorImg), *(graph->DepthImg));
		std::pair<GraspPose, float> Pair_tmp = std::make_pair(pos, Vec_Tmp[0]);
		m_pposes->push_back(Pair_tmp);
		if (m_ppos->second < Vec_Tmp[0])
		{
			m_ppos->second = Vec_Tmp[0];
			m_ppos->first = pos;
		}
	}
	delete graph->ColorImg;
	delete graph->DepthImg;
	delete graph;
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
		Graphics * graph = new Graphics;
		graph->ColorImg = new cv::Mat(COLORGRAPHHEIGHT, COLORGRAPHWIDTH, COLORFORMAT);
		graph->DepthImg = new cv::Mat(DEPTHGRAPHHEIGHT, DEPTHGRAPHWIDTH, COLORFORMAT);
		//save the subimage about the m_ppos to file
		if ((res = GetSubImage(&(m_ppos->first), m_pGraph, graph, false, false, 1)) != GT_RES_OK)
		{
			printf("Get Subimage failed with error: %02x.\n", res);
			delete graph->ColorImg;
			delete graph->DepthImg;
			delete graph;
			return res;
		}
		string FileNameColor,FileNameDepth;
		FileNameColor = IMAGEPATH;
		FileNameDepth = DEPTHPATH;
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
		FileNameDepth += str_tmp;
		FileNameDepth += ".jpg";
		cv::imwrite(FileNameColor, *(graph->ColorImg));
		cv::imwrite(FileNameDepth, *(graph->DepthImg));
		
		//save the predicted possibility to file
		std::ofstream out;
		out.open(PREDICTPATH, std::ios::app);
		out << str_tmp << " " << m_ppos->second << std::endl;
		out.close();

		res = m_pKinect->ColorDepth2Robot(m_ppos->first, *pos);
		m_pposes->clear();

		delete graph->ColorImg;
		delete graph->DepthImg;
		delete graph;
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
	//ȷ�����߳ߴ�һ��
	if (GraphIn->ColorImg->rows != COLORHEIGHT || GraphIn->ColorImg->cols != COLORWIDTH || GraphIn->DepthImg->rows != DEPTHHEIGHT || GraphIn->DepthImg->cols != DEPTHWIDTH)
	{
		printf("The size of GraphIn is not right.\n");
		return GT_RES_ERROR;
	}
	if (GraphOut->ColorImg->rows != COLORGRAPHHEIGHT || GraphOut->ColorImg->cols != COLORGRAPHWIDTH || GraphOut->DepthImg->rows != DEPTHGRAPHHEIGHT || GraphOut->DepthImg->cols != DEPTHGRAPHWIDTH)
	{
		printf("The size of GraphOut is not right.\n");
		return GT_RES_ERROR;
	}
	if (GraphIn->ColorImg->type() != GraphOut->ColorImg->type() || GraphIn->DepthImg->type() != GraphOut->DepthImg->type())
	{
		printf("The type of GraphIn and GraphOut is not consistent.\n");
		return GT_RES_ERROR;
	}
	//�߽���
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
	
	float Dx, Dy;
	GT_RES res = m_pKinect->Colorpos2Depthpos(pos->x, pos->y, Dx, Dy);
	if (res != GT_RES_OK)
	{
		printf("Colorpos convert to Depthpos failed with error:%02x.\n", res);
		return res;
	}
	else if (Dx<0 || Dx>=DEPTHWIDTH || Dy<0 || Dy>=DEPTHHEIGHT)
	{
		printf("depth image index is out of range.\n");
		return GT_RES_ERROR;
	}

	cv::Point2f Center1(pos->x, pos->y);
	cv::Point2f Center2(Dx, Dy);
	cv::Mat *pMat1 = GraphIn->ColorImg;
	cv::Mat *pMat2 = GraphIn->DepthImg;
	cv::Mat *pMat = NULL;

	if (fabs(pos->theta) > 1e-5)
	{
		pMat = new cv::Mat;
		cv::Mat RotationMat1 = cv::getRotationMatrix2D(Center1, pos->theta / PI * 180, COLORSCALE);
		cv::warpAffine(*pMat1, *pMat, RotationMat1, cv::Size(COLORWIDTH, COLORHEIGHT));
		pMat1 = pMat;

		pMat = new cv::Mat;
		cv::Mat RotationMat2 = cv::getRotationMatrix2D(Center2, pos->theta / PI * 180, COLORSCALE);
		cv::warpAffine(*pMat2, *pMat, RotationMat2, cv::Size(DEPTHWIDTH, DEPTHHEIGHT));
		pMat2 = pMat;
	}


	pMat = new cv::Mat;
	cv::Rect rect1(pos->x - COLORGRAPHWIDTH / 2.0, pos->y - COLORGRAPHHEIGHT / 2.0, COLORGRAPHWIDTH, COLORGRAPHHEIGHT);
	(*pMat1)(rect1).copyTo(*pMat);
	if (fabs(pos->theta) > 1e-5)
		delete pMat1;
	pMat1 = pMat;

	pMat = new cv::Mat;
	cv::Rect rect2(Dx - DEPTHGRAPHWIDTH / 2.0, Dy - DEPTHGRAPHHEIGHT / 2.0, DEPTHGRAPHWIDTH, DEPTHGRAPHHEIGHT);
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
		cv::flip(*pMat2, *(GraphOut->DepthImg), 0);
	}
	else
	{
		pMat1->copyTo(*(GraphOut->ColorImg));
		pMat2->copyTo(*(GraphOut->DepthImg));
	}
	delete pMat1;
	delete pMat2;
	return GT_RES_OK;
}