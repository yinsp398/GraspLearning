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
typedef struct _CameraSpacePoint CameraSpacePoint;

class NN
{
private:
	Classifier									*m_pClassifier;
	std::pair<GraspPose,float>					*m_ppos;
	std::vector<std::pair<GraspPose, float> >	*m_pposes;
	Graphics									*m_pGraph;
	time_t										*m_pUpdateTime;
	CoordinateMap								*m_pCoordinateMap;
	CameraSpacePoint							*m_pColorInCameraSpace;
	int											m_CloudWidthBias;
	int											m_CloudHeightBias;
	GT_RES										UpdateParam(std::string trained_file, std::string mean_file1, std::string mean_file2);
	GT_RES										GetRandomPoses(std::vector<GraspPose> *vec_pos);
	GT_RES										PosVecPush(std::pair<GraspPose, float> PosPair);
	GT_RES										PosVecClear(std::pair<GraspPose, float> *PosPair);
	bool										IsInLimits(GraspPose *pos);
	GT_RES										ColorDepth2Robot(const GraspPose posColor, Pose3D &posUR);
public:
	NN(const std::string & model_file, const std::string & trained_file, const std::string & mean_file1, const std::string & mean_file2);
	~NN();
	GT_RES										GetSubImage(GraspPose * const pos, Graphics* const GraphIn, Graphics *GraphOut, const bool flip, const bool contrast, const float contrast_scale);
	GT_RES										UpdateGraphics(Graphics * graph, CameraSpacePoint* ColorInCameraSpace, int CloudWidthBias, int CloudHeightBias);
	GT_RES										NNRun();
	GT_RES										GetPose(Pose3D *pos, const unsigned int ImgCnt);
	GT_RES										GetRandomPose(GraspPose * pos);
};