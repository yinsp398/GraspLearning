#include "ConvertDB.h"
#include "MergeMean.h"
#include <stdio.h>

unsigned int ImgCnt = 0;

int main()
{
	//�������� solver.prototxt�ļ�
	std::string caffemodelfile;
	//����caffeѵ������
	std::string TrainCmd = "caffe.exe train -solver solver.prototxt -weights " + caffemodelfile;
	system(TrainCmd.c_str());
	//�Ƿ���������
	//Convert DB

	//Get MeanDB
	

	return 0;
}