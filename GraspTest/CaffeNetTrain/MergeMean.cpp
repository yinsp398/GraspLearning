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

void MergeMean(const caffe::BlobProto blob_new, const unsigned int CntImg, const char* mean_filename, const unsigned int CntMean)
{
	if (CntImg == 0 || CntImg <= CntMean)
	{
		std::cout << "CntImg and CntMean value error!" << std::endl;
		return;
	}
	caffe::BlobProto blob_proto;
	std::fstream fp;
	fp.open(mean_filename, std::ios::in);													//判断文件是否存在
	if (!fp)
	{
		std::cout << "have no mean file, then we will create it" << std::endl;
		caffe::WriteProtoToBinaryFile(blob_new, mean_filename);								//文件不存在就是直接把blob_new创建并写入meanfile
	}
	else
	{
		bool res = caffe::ReadProtoFromBinaryFile(mean_filename, &blob_proto);
		if (!res)
		{
			CHECK_EQ(blob_new.data_size(), blob_proto.data_size());
			for (size_t i = 0; i < blob_proto.data_size(); i++)
			{
				blob_proto.set_data(i, (blob_new.data(i) / (CntImg) * (CntImg - CntMean) + blob_proto.data(i) / (CntImg) * (CntMean)));
			}
			caffe::WriteProtoToBinaryFile(blob_proto, mean_filename);
		}
	}
}