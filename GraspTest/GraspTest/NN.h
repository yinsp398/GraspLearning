#pragma once

#include "Common.h"
#include <opencv2/core/core.hpp>
#include <utility>
#include <vector>
#include <string>

class Pose3D;
class Classifier;
class CoordinateMap;
struct Graphics;
class KinectDriver;

class NN
{
private:
	Classifier									*m_pClassifier;
	std::pair<GraspPose,float>					*m_ppos;
	std::vector<std::pair<GraspPose, float> >	*m_pposes;
	Graphics									*m_pGraph;
	time_t										*m_pUpdateTime;
	CoordinateMap								*m_pCoordinateMap;
	KinectDriver								*m_pKinect;
	unsigned int								ImageCnt;
	GT_RES										UpdateParam(std::string trained_file, std::string mean_file);
	GT_RES										GetRandomPose(GraspPose * pos);
	GT_RES										GetRandomPoses(std::vector<GraspPose> *vec_pos);
	GT_RES										GetSubImage(GraspPose *pos, cv::Mat *ImgIn, cv::Mat *ImgOut);

public:
	NN(const std::string & model_file, const std::string & trained_file, const std::string & mean_file, KinectDriver * Kinect = NULL);
	~NN();
	GT_RES										UpdateGraphics(Graphics * graph);
	GT_RES										NNRun();
	GT_RES										GetPose(Pose3D *pos);
};