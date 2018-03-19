#pragma once
#include <caffe\caffe.hpp>

void MergeMean(const caffe::BlobProto blob_new, const unsigned int CntImg, const char* mean_filename, const unsigned int CntMean);