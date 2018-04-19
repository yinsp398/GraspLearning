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
#define		CONFIGFILE			"..\\model\\config.txt"
#define		BATCHSIZE			64

bool ReadConfig(const std::string file, unsigned long &FilePos, unsigned int &ImgCnt, double &base_lr, unsigned int &iteration, unsigned int &Label)
{
	std::fstream fpconfig;
	fpconfig.open(file, std::ios::in);
	if (fpconfig)
	{
		std::string line;
		size_t pos;
		std::getline(fpconfig, line);
		pos = line.find_last_of(":");
		FilePos = atol(line.substr(pos + 1).c_str());
		std::getline(fpconfig, line);
		pos = line.find_last_of(":");
		ImgCnt = atoi(line.substr(pos + 1).c_str());
		std::getline(fpconfig, line);
		pos = line.find_last_of(":");
		base_lr = atof(line.substr(pos + 1).c_str());
		std::getline(fpconfig, line);
		pos = line.find_last_of(":");
		iteration = atoi(line.substr(pos + 1).c_str());
		std::getline(fpconfig, line);
		pos = line.find_last_of(":");
		Label = atoi(line.substr(pos + 1).c_str());
		fpconfig.close();
	}
	else
	{
		return false;
	}
	return true;
}
bool WriteConfig(const std::string file, const unsigned long FilePos, const unsigned int ImgCnt, const double base_lr, int iteration, unsigned int Label)
{
	std::fstream fpconfig;
	fpconfig.open(file, std::ios::out);
	if (fpconfig)
	{
		fpconfig << "FilePos:" << FilePos << std::endl;
		fpconfig << "ImgCnt:" << ImgCnt << std::endl;
		fpconfig << "base_lr:" << base_lr << std::endl;
		fpconfig << "iteration:" << iteration << std::endl;
		fpconfig << "Success:" << Label << std::endl;
		fpconfig.close();
	}
	else
	{
		return false;
	}
	return true;
}

int main()
{
	STARTUPINFO si;
	PROCESS_INFORMATION pi;
	ZeroMemory(&si, sizeof(si));
	si.cb = sizeof(si);
	ZeroMemory(&pi, sizeof(pi));

	std::string caffemodelfile = "..\\model\\Grasp_iter_1000.caffemodel";
	
	unsigned long FilePos = 0;
	unsigned int ImgCnt = 0;
	double base_lr = 0.001;
	unsigned iteration = 0;
	unsigned Label = 0;
	ReadConfig(CONFIGFILE, FilePos, ImgCnt, base_lr, iteration, Label);
	double momentum = 0.9;
	for (int i = 0; i < 100; i++, iteration++)
	{
		std::cout << "iteration:" << iteration << std::endl;
		std::cout << "convert jpg to lmdb......" << std::endl;
		//Convert DB
		unsigned int NewCnt;
		NewCnt = SaveConvert(IMAGEPATH, LABELFILE, DBCOLORPATH, DBDEPTHPATH, "lmdb", FilePos, Label, MEANCOLORNEWFILE, MEANDEPTHNEWFILE);
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

		WriteConfig(CONFIGFILE, FilePos, ImgCnt, base_lr, iteration, Label);
		//生成配置 solver.prototxt文件
		std::cout << "Writing solver.prototxt......" << std::endl;
		std::fstream fpsolver;
		fpsolver.open("..\\model\\solver.prototxt", std::ios::out);
		fpsolver << "net:" << "\"../model/Grasp_train_test.prototxt\"" << std::endl;
		fpsolver << "test_iter:" << ImgCnt / BATCHSIZE << std::endl;
		fpsolver << "test_interval:" << 1000 << std::endl;
		fpsolver << "base_lr:" << base_lr << std::endl;
		fpsolver << "momentum:" << momentum << std::endl;
		fpsolver << "lr_policy:\"fixed\"" << std::endl;
		fpsolver << "display:1000" << std::endl;
		fpsolver << "max_iter:" << 1000 << std::endl;
		fpsolver << "snapshot:" << 1000 << std::endl;
		fpsolver << "snapshot_prefix:\"../model/Grasp\"" << std::endl;
		fpsolver << "solver_mode:GPU" << std::endl;
		fpsolver.close();
		//调用caffe训练数据
		std::cout << "Training......" << std::endl;
		std::string TrainCmd;
		if (caffemodelfile.size() == 0)
		{
			TrainCmd = "..\\caffe_bin\\caffe-d.exe train -solver ..\\model\\solver.prototxt";
		}
		else
		{
			TrainCmd = "..\\caffe_bin\\caffe-d.exe train -solver ..\\model\\solver.prototxt -weights " + caffemodelfile;
		}
		LPTSTR szCmdline = _tcsdup(TEXT(TrainCmd.c_str()));
		if (!CreateProcess(NULL, szCmdline, NULL, NULL, FALSE, 0, NULL, NULL, &si, &pi))
		{
			printf("CreateProcess failed (%d).\n", GetLastError());
			WriteConfig(CONFIGFILE, FilePos, ImgCnt, base_lr, iteration, Label);
			return 1;
		}
		WaitForSingleObject(pi.hProcess, INFINITE);
		CloseHandle(pi.hProcess);
		CloseHandle(pi.hThread);
		caffemodelfile = "..\\model\\Grasp_iter_1000.caffemodel";
		
	}
	WriteConfig(CONFIGFILE, FilePos, ImgCnt, base_lr, iteration, Label);
	return 0;
}