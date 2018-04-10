#include "ConvertDB.h"
#include "MergeMean.h"
#include <tchar.h>
#include <fstream>
#include <stdio.h>
#include <windows.h>

#define		LABELFILE			"..\\imageset\\Result.txt"
#define		IMAGEPATH			"..\\imageset\\"
#define		DBCOLORPATH			"..\\imageset\\lmdbcolor"
#define		DBDEPTHPATH			"..\\imageset\\lmdbdepth"
#define		MEANCOLORNEWFILE	"..\\imageset\\MeanFile\\meancolor2.binaryproto"
#define		MEANCOLORFILE		"..\\imageset\\MeanFile\\meancolor.binaryproto"
#define		MEANDEPTHNEWFILE	"..\\imageset\\MeanFile\\meandepth2.binaryproto"
#define		MEANDEPTHFILE		"..\\imageset\\MeanFile\\meandepth.binaryproto"
#define		BATCHSIZE			64
int main()
{
	STARTUPINFO si;
	PROCESS_INFORMATION pi;
	ZeroMemory(&si, sizeof(si));
	si.cb = sizeof(si);
	ZeroMemory(&pi, sizeof(pi));

	std::string caffemodelfile;
	unsigned int FilePos = 0;
	unsigned int ImgCnt = 0;
	double base_lr = 0.01;
	double momentum = 0.9;
	unsigned iteration = 0;
	for (int iteration = 0; iteration < 10000; iteration++)
	{
		std::cout << "iteration:" << iteration << std::endl;
		std::cout << "convert jpg to lmdb......" << std::endl;
		//Convert DB
		unsigned int NewCnt;
		NewCnt = SaveConvert(IMAGEPATH, LABELFILE, DBCOLORPATH, DBDEPTHPATH, "lmdb", FilePos, MEANCOLORNEWFILE, MEANDEPTHNEWFILE);
		std::cout << "merge new meanfile with meanfile......" << std::endl;
		//Merge MeanDB
		if (!MergeMean(MEANCOLORFILE, ImgCnt, MEANCOLORNEWFILE, NewCnt))
		{
			printf("merge meanfile failed!\n");
		}
		if (!MergeMean(MEANDEPTHFILE, ImgCnt, MEANDEPTHNEWFILE, NewCnt))
		{
			printf("merge meanfile failed!\n");
		}
		ImgCnt += NewCnt;
		//生成配置 solver.prototxt文件
		std::cout << "Writing solver.prototxt......" << std::endl;
		std::fstream fpsolver;
		fpsolver.open("..\\imageset\\solver.prototxt", std::ios::out);
		fpsolver << "net:" << "\"..\\imageset\\Grasp_train_test.prototxt\"" << std::endl;
		fpsolver << "test_iter:" << ImgCnt / BATCHSIZE << std::endl;
		fpsolver << "test_interval:" << 10000 << std::endl;
		fpsolver << "base_lr:" << base_lr << std::endl;
		fpsolver << "momentum:" << momentum << std::endl;
		fpsolver << "lr_policy:\"fixed\"" << std::endl;
		fpsolver << "display:1000" << std::endl;
		fpsolver << "max_iter:" << 10000 << std::endl;
		fpsolver << "snapshot:" << 10000 << std::endl;
		fpsolver << "snapshot_prefix:\"..\\imageset\\Grasp_" << iteration << "\"" << std::endl;
		fpsolver << "solver_mode:GPU" << std::endl;
		fpsolver.close();
		//调用caffe训练数据
		std::cout << "Training......" << std::endl;
		std::string TrainCmd;
		if (caffemodelfile.size() == 0)
		{
			TrainCmd = "..\\caffe_bin\\caffe-d.exe train -solver solver.prototxt";
		}
		else
		{
			TrainCmd = "..\\caffe_bin\\caffe-d.exe train -solver solver.prototxt -weights " + caffemodelfile;
		}
		LPTSTR szCmdline = _tcsdup(TEXT(TrainCmd.c_str()));
		if (!CreateProcess(NULL, szCmdline, NULL, NULL, FALSE, 0, NULL, NULL, &si, &pi))
		{
			printf("CreateProcess failed (%d).\n", GetLastError());
			return 1;
		}
		WaitForSingleObject(pi.hProcess, INFINITE);
		CloseHandle(pi.hProcess);
		CloseHandle(pi.hThread);
		caffemodelfile = "..\\imageset\\Grasp_" + iteration;
		caffemodelfile += "_iter_10000.caffemodelfile";
		
	}
	return 0;
}