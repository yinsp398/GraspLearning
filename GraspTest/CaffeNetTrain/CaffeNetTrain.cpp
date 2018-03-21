#include "ConvertDB.h"
#include "MergeMean.h"
#include <stdio.h>

unsigned int ImgCnt = 0;

int main()
{
	//生成配置 solver.prototxt文件
	std::string caffemodelfile;
	//调用caffe训练数据
	std::string TrainCmd = "caffe.exe train -solver solver.prototxt -weights " + caffemodelfile;
	system(TrainCmd.c_str());
	//是否阻塞？？
	//Convert DB

	//Get MeanDB
	

	return 0;
}