#pragma once
#include <caffe\caffe.hpp>

bool MergeMean(const char* mean_filename, const unsigned int CntImg, const char* newmean_filename, const unsigned int NewCnt);