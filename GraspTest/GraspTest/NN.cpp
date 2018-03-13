
#include "NN.h"
#include "Classification.h"
#include "CoordinateMap.h"
#include <Kinect.h>

NN::NN(	const std::string & model_file,
		const std::string & trained_file,
		const std::string & mean_file)
{
	//m_pClassifier = new Classifier(model_file, trained_file, mean_file );
	m_ppos = new std::pair<GraspPose, float>;
	m_pposes = new std::vector<std::pair<GraspPose, float> >;
	srand((unsigned)time(NULL));											//initialize the rand seed by time stamp
	m_pUpdateTime = new time_t;
	*m_pUpdateTime = time(NULL);
	ImageCnt = 0;
	m_pGraph = new Graphics;
	m_pGraph->DepthImg = new cv::Mat(DEPTHHEIGHT, DEPTHWIDTH, DEPTHFORMAT);
	m_pGraph->ColorImg = new cv::Mat(COLORHEIGHT, COLORWIDTH, COLORFORMAT);
	m_pGraph->DepthInColorImg = new cv::Mat(COLORHEIGHT, COLORWIDTH, DEPTHFORMAT);

}

NN::~NN()
{
	delete m_pClassifier;
	delete m_ppos;
	delete m_pposes;
	delete m_pUpdateTime;
	delete m_pGraph->DepthImg;
	delete m_pGraph->ColorImg;
	delete m_pGraph->DepthInColorImg;
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
	// deeply copy from graph to m_pGraph
	memcpy(m_pGraph->ColorImg, graph->ColorImg, sizeof(graph->ColorImg));
	memcpy(m_pGraph->DepthImg, graph->DepthImg, sizeof(graph->DepthImg));
	memcpy(m_pGraph->DepthInColorImg, graph->DepthInColorImg, sizeof(graph->DepthInColorImg));

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
	cv::Mat *pImage = m_pGraph->ColorImg;
	GraspPose pos;
	for (int i = 0; i < 50; i++)
	{
		if ((res = GetRandomPose(&pos)) != GT_RES_OK)
		{
			printf("Get random pose failed with error: %02x.\n", res);
			return res;
		}
		cv::Mat SubImage(COLORGRAPHHEIGHT, COLORGRAPHWIDE, pImage->type(), cv::Scalar(0, 0, 0));
		if ((res = GetSubImage(&pos, pImage, &SubImage)) != GT_RES_OK)
		{
			printf("Get Subimage failed with error: %02x.\n", res);
			return res;
		}
		std::vector<float> Vec_Tmp = m_pClassifier->Predict(SubImage);
		std::pair<GraspPose, float> Pair_tmp = std::make_pair(pos, Vec_Tmp[0]);
		m_pposes->push_back(Pair_tmp);
		if (m_ppos->second < Vec_Tmp[0])
		{
			m_ppos->second = Vec_Tmp[0];
			m_ppos->first = pos;
		}
	}

	return GT_RES_OK;
}

//Get graph by the center of pose
GT_RES	NN::GetSubImage(GraspPose *pos, cv::Mat *ImgIn, cv::Mat *ImgOut)
{
	if (pos == NULL || ImgIn == NULL || ImgOut == NULL)
	{
		printf("The pointer to pos or image is NULL.\n");
		return GT_RES_ERROR;
	}
	//确保二者尺寸一致
	if (ImgIn->type() != ImgOut->type())
	{
		printf("The size or type of ImgIn and ImgOut are not consistent.\n");
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

	cv::Point2f Center(pos->x, pos->y);
	cv::Mat RotationMat = cv::getRotationMatrix2D(Center, pos->theta, COLORSCALE);
	cv::Mat NewImg(ImgIn->rows, ImgIn->cols, ImgIn->type(), cv::Scalar(0, 0, 0));
	cv::warpAffine(*ImgIn, NewImg, RotationMat, cv::Size(ImgIn->cols, ImgIn->rows));
	cv::Rect rect(pos->x - COLORGRAPHWIDE / 2, pos->y - COLORGRAPHHEIGHT, COLORGRAPHWIDE, COLORGRAPHHEIGHT);
	cv::Mat SubImg = NewImg(rect);
	SubImg.copyTo(*ImgOut);
	return GT_RES_OK;
}

//Get random pose one by one (algorithm 1)
GT_RES	NN::GetRandomPose(GraspPose * pos)
{
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
GT_RES	NN::GetPose(Pose3D *pos)
{
	GT_RES res;
	if (m_ppos)
	{
		//save the subimage about the m_ppos to file
		cv::Mat* pImage =m_pGraph->ColorImg;
		cv::Mat SubImage(COLORGRAPHHEIGHT, COLORGRAPHWIDE, pImage->type(), cv::Scalar(0, 0, 0));
		if ((res = GetSubImage(&m_ppos->first, pImage, &SubImage)) != GT_RES_OK)
		{
			printf("Get Subimage failed with error: %02x.\n", res);
			return res;
		}
		string FileName;
		FileName = IMAGEPATH;
		std::stringstream ss;
		string str_tmp;
		ss << ImageCnt;
		ss >> str_tmp;
		FileName += str_tmp;
		FileName += ".jpg";
		cv::imwrite(FileName,SubImage);
		
		//save the predicted possibility to file
		std::ofstream out;
		out.open(PREDICTPATH, std::ios::out);
		out << m_ppos->second<<std::endl;
		out.close();

		float depth = m_pGraph->DepthInColorImg->at<uchar>(m_ppos->first.y, m_ppos->first.x);
		res = m_pCoordinateMap->ColorDepth2Robot(m_ppos->first,depth,*pos);
		m_pposes->clear();
		ImageCnt++;
		return GT_RES_OK;
	}
	else
	{
		printf("Have not initialized the NN\n");
		return GT_RES_ERROR;
	}
}