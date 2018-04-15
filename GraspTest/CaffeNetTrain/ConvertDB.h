#pragma once
#include <caffe\caffe.hpp>
#include <string>
unsigned int SaveConvert(const char* image_filepath, const char* label_filename,
	const char* dbColor_path, const char* dbDepth_path,
	const std::string& db_backend, unsigned long &FilePos,
	const std::string &meancolorfile, const std::string &meandepthfile);
bool addto_dataset(const char* image_rootpath, const char* image_file, const int label, const char* db_path, const std::string& db_backend);
void convert_dataset(const char* image_filepath, const char* label_filename, const char* db_path, const std::string& db_backend);
void DatumToMat(const caffe::Datum* datum, cv::Mat &cv_img);
void getimage_dataset(const char* dp_path, const char* db_backend, const char* image_filepath, const char* labelpath);