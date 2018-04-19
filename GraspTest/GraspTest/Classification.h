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
typedef std::pair<string, float> Prediction;

class Classifier {
public:
	Classifier(const string& model_file,
		const string& trained_file,
		const string& mean_file1,
		const string& mean_file2);
	void Updatefile(const string& mean_file1, const string& mean_file2, const string& trained_file);
	std::vector<float> Predict(const cv::Mat& colorimg, const cv::Mat& CloudPointsimg);
	std::vector<Prediction> Classify(const cv::Mat& imgColor, const cv::Mat& imgCloudPoints, int N);
private:
	
	void SetMean(const string& mean_file1, const string& mean_file2);

	void WrapInputLayer(std::vector<cv::Mat>* input_channels1, std::vector<cv::Mat>* input_channels2);

	void Preprocess(const cv::Mat& Colorimg, const cv::Mat& CloudPointsimg, std::vector<cv::Mat>* input_channels1, std::vector<cv::Mat>* input_channels2);

private:
	caffe::shared_ptr<caffe::Net<float> > net_;
	cv::Size input_geometry_1;
	cv::Size input_geometry_2;
	int num_channels_1;
	int num_channels_2;
	cv::Mat mean_1;
	cv::Mat mean_2;
	std::vector<string> labels_;
};
