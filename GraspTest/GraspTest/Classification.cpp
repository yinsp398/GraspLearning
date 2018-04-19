
#include "Classification.h"



Classifier::Classifier(const string& model_file,
	const string& trained_file,
	const string& mean_file1,
	const string& mean_file2) {
#ifdef CPU_ONLY
	caffe::Caffe::set_mode(caffe::Caffe::CPU);
#else
	caffe::Caffe::set_mode(caffe::Caffe::GPU);
#endif

	/* Load the network. */
	net_.reset(new caffe::Net<float>(model_file, caffe::TEST));
	net_->CopyTrainedLayersFrom(trained_file);
	int test1 = net_->num_inputs();
	int test2 = net_->num_outputs();
	CHECK_EQ(net_->num_inputs(), 2) << "Network should have exactly one input.";
	CHECK_EQ(net_->num_outputs(), 1) << "Network should have exactly one output.";

	caffe::Blob<float>* input_layer1 = net_->input_blobs()[0];
	caffe::Blob<float>* input_layer2 = net_->input_blobs()[1];

	num_channels_1 = input_layer1->channels();
	num_channels_2 = input_layer2->channels();
	CHECK(num_channels_1 == 3 || num_channels_1 == 1)
		<< "Input layer 1 should have 1 or 3 channels.";
	CHECK(num_channels_2 == 3 || num_channels_2 == 1)
		<< "Input layer 2 should have 1 or 3 channels.";
	input_geometry_1 = cv::Size(input_layer1->width(), input_layer1->height());
	input_geometry_2 = cv::Size(input_layer2->width(), input_layer2->height());

	/* Load the binaryproto mean file. */
	SetMean(mean_file1, mean_file2);

}

void Classifier::Updatefile(const string& mean_file1, const string& mean_file2, const string& trained_file)
{
	net_->CopyTrainedLayersFrom(trained_file);
	SetMean(mean_file1,mean_file2);
}

static bool PairCompare(const std::pair<float, int>& lhs,
	const std::pair<float, int>& rhs) {
	return lhs.first > rhs.first;
}

/* Return the indices of the top N values of vector v. */
static std::vector<int> Argmax(const std::vector<float>& v, int N) {
	std::vector<std::pair<float, int> > pairs;
	for (size_t i = 0; i < v.size(); ++i)
		pairs.push_back(std::make_pair(v[i], static_cast<int>(i)));
	std::partial_sort(pairs.begin(), pairs.begin() + N, pairs.end(), PairCompare);

	std::vector<int> result;
	for (int i = 0; i < N; ++i)
		result.push_back(pairs[i].second);
	return result;
}

/* Return the top N predictions. */
std::vector<Prediction> Classifier::Classify(const cv::Mat& imgColor, const cv::Mat& imgCloudPoints, int N) {
	std::vector<float> output = Predict(imgColor,imgCloudPoints);

	N = std::min<int>(labels_.size(), N);
	std::vector<int> maxN = Argmax(output, N);
	std::vector<Prediction> predictions;
	for (int i = 0; i < N; ++i) {
		int idx = maxN[i];
		predictions.push_back(std::make_pair(labels_[idx], output[idx]));
	}

	return predictions;
}

/* Load the mean file in binaryproto format. */
void Classifier::SetMean(const string& mean_file1,const string& mean_file2) {
	caffe::BlobProto blob_proto1, blob_proto2;
	ReadProtoFromBinaryFileOrDie(mean_file1.c_str(), &blob_proto1);
	ReadProtoFromBinaryFileOrDie(mean_file2.c_str(), &blob_proto2);

	/* Convert from BlobProto to Blob<float> */
	caffe::Blob<float> mean_blob1, mean_blob2;
	mean_blob1.FromProto(blob_proto1);
	mean_blob2.FromProto(blob_proto2);
	CHECK_EQ(mean_blob1.channels(), num_channels_1)
		<< "Number of channels of mean file 1 doesn't match input layer.";
	CHECK_EQ(mean_blob2.channels(), num_channels_2)
		<< "Number of channels of mean file 2 doesn't match input layer.";

	/* The format of the mean file is planar 32-bit float BGR or grayscale. */
	std::vector<cv::Mat> channels1, channels2;
	float* data1 = mean_blob1.mutable_cpu_data();
	float* data2 = mean_blob2.mutable_cpu_data();
	for (int i = 0; i < num_channels_1; ++i) {
		/* Extract an individual channel. */
		cv::Mat channel(mean_blob1.height(), mean_blob1.width(), CV_32FC1, data1);
		channels1.push_back(channel);
		data1 += mean_blob1.height() * mean_blob1.width();
	}
	for (int i = 0; i < num_channels_2; ++i) {
		/* Extract an individual channel. */
		cv::Mat channel(mean_blob2.height(), mean_blob2.width(), CV_32FC1, data2);
		channels2.push_back(channel);
		data2 += mean_blob2.height() * mean_blob2.width();
	}

	/* Merge the separate channels into a single image. */
	cv::Mat mean1, mean2;
	cv::merge(channels1, mean1);
	cv::merge(channels2, mean2);

	/* Compute the global mean pixel value and create a mean image
	* filled with this value. */
	cv::Scalar channel_mean1 = cv::mean(mean1);
	cv::Scalar channel_mean2 = cv::mean(mean2);
	mean_1 = cv::Mat(input_geometry_1, mean1.type(), channel_mean1);
	mean_2 = cv::Mat(input_geometry_2, mean2.type(), channel_mean2);
}

std::vector<float> Classifier::Predict(const cv::Mat& Colorimg, const cv::Mat& CloudPointsimg) {
	caffe::Blob<float>* input_layer1 = net_->input_blobs()[0];
	caffe::Blob<float>* input_layer2 = net_->input_blobs()[1];
	input_layer1->Reshape(1, num_channels_1,
		input_geometry_1.height, input_geometry_1.width);
	input_layer2->Reshape(1, num_channels_2,
		input_geometry_2.height, input_geometry_2.width);
	/* Forward dimension change to all layers. */
	net_->Reshape();

	std::vector<cv::Mat> input_channels1, input_channels2;
	WrapInputLayer(&input_channels1, &input_channels2);

	Preprocess(Colorimg, CloudPointsimg, &input_channels1, &input_channels2);

	net_->Forward();

	/* Copy the output layer to a std::vector */
	caffe::Blob<float>* output_layer = net_->output_blobs()[0];
	const float* begin = output_layer->cpu_data();
	const float* end = begin + output_layer->channels();
	return std::vector<float>(begin, end);
}

/* Wrap the input layer of the network in separate cv::Mat objects
* (one per channel). This way we save one memcpy operation and we
* don't need to rely on cudaMemcpy2D. The last preprocessing
* operation will write the separate channels directly to the input
* layer. */
void Classifier::WrapInputLayer(std::vector<cv::Mat>* input_channels1, std::vector<cv::Mat>* input_channels2) {
	caffe::Blob<float>* input_layer1 = net_->input_blobs()[0];
	caffe::Blob<float>* input_layer2 = net_->input_blobs()[1];

	int width1 = input_layer1->width();
	int height1 = input_layer1->height();
	float* input_data1 = input_layer1->mutable_cpu_data();
	for (int i = 0; i < input_layer1->channels(); ++i) {
		cv::Mat channel(height1, width1, CV_32FC1, input_data1);
		input_channels1->push_back(channel);
		input_data1 += width1 * height1;
	}
	int width2 = input_layer2->width();
	int height2 = input_layer2->height();
	float* input_data2 = input_layer2->mutable_cpu_data();
	for (int i = 0; i < input_layer2->channels(); ++i) {
		cv::Mat channel(height2, width2, CV_32FC1, input_data2);
		input_channels2->push_back(channel);
		input_data2 += width2 * height2;
	}
}

void Classifier::Preprocess(const cv::Mat& Colorimg, const cv::Mat& CloudPointsimg, std::vector<cv::Mat>* input_channels1, std::vector<cv::Mat>* input_channels2)
{
	/* Convert the input image to the input image format of the network. */
	cv::Mat sample1;
	if (Colorimg.channels() == 3 && num_channels_1 == 1)
		cv::cvtColor(Colorimg, sample1, cv::COLOR_BGR2GRAY);
	else if (Colorimg.channels() == 4 && num_channels_1 == 1)
		cv::cvtColor(Colorimg, sample1, cv::COLOR_BGRA2GRAY);
	else if (Colorimg.channels() == 4 && num_channels_1 == 3)
		cv::cvtColor(Colorimg, sample1, cv::COLOR_BGRA2BGR);
	else if (Colorimg.channels() == 1 && num_channels_1 == 3)
		cv::cvtColor(Colorimg, sample1, cv::COLOR_GRAY2BGR);
	else
		sample1 = Colorimg;

	cv::Mat sample_resized1;
	if (sample1.size() != input_geometry_1)
		cv::resize(sample1, sample_resized1, input_geometry_1);
	else
		sample_resized1 = sample1;

	cv::Mat sample_float1;
	if (num_channels_1 == 3)
		sample_resized1.convertTo(sample_float1, CV_32FC3);
	else
		sample_resized1.convertTo(sample_float1, CV_32FC1);

	cv::Mat sample_normalized1;
	cv::subtract(sample_float1, mean_1, sample_normalized1);

	/* This operation will write the separate BGR planes directly to the
	* input layer of the network because it is wrapped by the cv::Mat
	* objects in input_channels. */
	cv::split(sample_normalized1, *input_channels1);

	CHECK(reinterpret_cast<float*>(input_channels1->at(0).data)
		== net_->input_blobs()[0]->cpu_data())
		<< "Input channels are not wrapping the input layer of the network.";
	/* Convert the input image to the input image format of the network. */
	cv::Mat sample2;
	if (CloudPointsimg.channels() == 3 && num_channels_2 == 1)
		cv::cvtColor(CloudPointsimg, sample2, cv::COLOR_BGR2GRAY);
	else if (CloudPointsimg.channels() == 4 && num_channels_2 == 1)
		cv::cvtColor(CloudPointsimg, sample2, cv::COLOR_BGRA2GRAY);
	else if (CloudPointsimg.channels() == 4 && num_channels_2 == 3)
		cv::cvtColor(CloudPointsimg, sample2, cv::COLOR_BGRA2BGR);
	else if (CloudPointsimg.channels() == 1 && num_channels_2 == 3)
		cv::cvtColor(CloudPointsimg, sample2, cv::COLOR_GRAY2BGR);
	else
		sample2 = CloudPointsimg;

	cv::Mat sample_resized2;
	if (sample2.size() != input_geometry_2)
		cv::resize(sample2, sample_resized2, input_geometry_2);
	else
		sample_resized2 = sample2;

	cv::Mat sample_float2;
	if (num_channels_2 == 3)
		sample_resized2.convertTo(sample_float2, CV_32FC3);
	else
		sample_resized2.convertTo(sample_float2, CV_32FC1);

	cv::Mat sample_normalized2;
	cv::subtract(sample_float2, mean_2, sample_normalized2);

	/* This operation will write the separate BGR planes directly to the
	* input layer of the network because it is wrapped by the cv::Mat
	* objects in input_channels. */
	cv::split(sample_normalized2, *input_channels2);

	CHECK(reinterpret_cast<float*>(input_channels2->at(0).data)
		== net_->input_blobs()[1]->cpu_data())
		<< "Input channels are not wrapping the input layer of the network.";
}

