#pragma once
#include <caffe/caffe.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <algorithm>
#include <iosfwd>
#include <memory>
#include <string>
#include <utility>
#include <vector>


//using namespace caffe;  // NOLINT(build/namespaces)
using std::string;

/* Pair (label, confidence) representing a prediction. */
//typedef std::pair<string, float> Prediction;

class Classifier {
public:
	Classifier(const string& model_file,
		const string& trained_file,
		const string& mean_file);
	void Updatefile(const string& mean_file, const string& trained_file);
	std::vector<float> Predict(const cv::Mat& colorimg, const cv::Mat& CloudPointsimg);
private:
	
	void SetMean(const string& mean_file);

	void WrapInputLayer(std::vector<cv::Mat>* input_channels);

	void Preprocess(const cv::Mat& img,
		std::vector<cv::Mat>* input_channels);

private:
	caffe::shared_ptr<caffe::Net<float> > net_;
	cv::Size input_geometry_;
	int num_channels_;
	cv::Mat mean_;
	std::vector<string> labels_;
};
