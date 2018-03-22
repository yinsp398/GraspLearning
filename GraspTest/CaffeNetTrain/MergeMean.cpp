#include "MergeMean.h"
#include <stdint.h>
#include <algorithm>
#include <string>
#include <utility>
#include <vector>
#include <fstream>

#include "boost/scoped_ptr.hpp"
#include "gflags/gflags.h"
#include "glog/logging.h"

#include "caffe/proto/caffe.pb.h"
#include "caffe/util/db.hpp"
#include "caffe/util/io.hpp"

bool MergeMean(const char* mean_filename, const unsigned int CntImg, const char* newmean_filename, const unsigned int NewCnt)
{
	if (0 == NewCnt)
	{
		std::cout << "CntImg and CntMean value error!" << std::endl;
		return false;
	}
	caffe::BlobProto blob_proto;
	caffe::BlobProto blob_new;
	std::fstream fp;
	fp.open(mean_filename, std::ios::in);													//判断文件是否存在
	if (!fp)
	{
		std::cout << "have no mean file, then we will create it" << std::endl;
		bool res = caffe::ReadProtoFromBinaryFile(newmean_filename, &blob_new);
		if (res)
		{
			caffe::WriteProtoToBinaryFile(blob_new, mean_filename);								//文件不存在就是直接把blob_new创建并写入meanfile
			return true;
		}
	}
	else
	{
		fp.close();
		bool res1 = caffe::ReadProtoFromBinaryFile(mean_filename, &blob_proto);
		bool res2 = caffe::ReadProtoFromBinaryFile(newmean_filename, &blob_new);
		if ((res1) && (res2))
		{
			CHECK_EQ(blob_new.data_size(), blob_proto.data_size());
			for (size_t i = 0; i < blob_proto.data_size(); i++)
			{
				blob_proto.set_data(i, (blob_new.data(i) * (NewCnt) / (CntImg + NewCnt) + blob_proto.data(i) * (CntImg) / (CntImg + NewCnt)));
			}
			caffe::WriteProtoToBinaryFile(blob_proto, mean_filename);
			return true;
		}
	}
	return false;
}