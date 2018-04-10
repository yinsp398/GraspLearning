// CoordinateCalibration.cpp : 定义控制台应用程序的入口点。
//
#include <Common.h>
#include <opencv2\opencv.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include <KinectDriver.h>
#include <Kinect.h>
#include <UR5SocketCom.h>
#include <UR5SocketComCodes.h>
#include <utils.h>
#include <fstream>
#include <iomanip>

#define _DEBUG_PRINT_
#define _MAX_RADIUS_		200
#define _MIN_RADIUS_		50
#define _DETAL_				5

struct Pose {
	float x;
	float y;
	float z;
	Pose()
	{
		x = 0;
		y = 0;
		z = 0;
	}
	Pose(const float x1, const float y1, const float z1)
	{
		x = x1;
		y = y1;
		z = z1;
	}
	Pose(const Pose &pos)
	{
		x = pos.x;
		y = pos.y;
		z = pos.z;
	}
	void GetAverage(Pose pos)
	{
		x = (pos.x + x) / 2;
		y = (pos.y + y) / 2;
		z = (pos.z + z) / 2;
	}
	void Set(const float x1,const float y1,const float z1)
	{
		x = x1;
		y = y1;
		z = z1;
	}
	Pose operator+(const Pose &other)
	{
		Pose pos;
		pos.x = this->x + other.x;
		pos.y = this->y + other.y;
		pos.z = this->z + other.z;
		return pos;
	}
	Pose operator-(const Pose &other)
	{
		Pose pos;
		pos.x = this->x - other.x;
		pos.y = this->y - other.y;
		pos.z = this->z - other.z;
		return pos;
	}
	Pose operator/(const float &num)
	{
		Pose pos;
		if (num == 0.0)
			return pos;
		pos.x = this->x / num;
		pos.y = this->y / num;
		pos.z = this->z / num;
		return pos;
	}
};

KinectDriver *m_pKinect;
UR5SocketCom *m_pUR5;
Graphics	 *m_pGraph;
int			count;

bool Init();
bool GetCenterPos(Pose &pos);
bool GetRobotPos(Pose &pos);
bool GetCirclePos(cv::Mat image,Pose &pos);
bool GetLinePos(cv::Mat Image, Pose &pos);
bool OpenUR5();
Pose CrossPoint(cv::Vec2f line1, cv::Vec2f line2);
bool GetBYTEformat(cv::Mat *DepthImg, cv::Mat *OutImg);
bool IsInLimit(CameraSpacePoint pos);
bool MovetoPos(Pose3D URpos);
bool ResetPos();
bool TestSaveMoreThanOneImage();
bool TestCalibration();
bool TestMoveCalibration();
bool TestShowImage();
bool TestVerifyTransMat();
bool TestErrorUR5(int Num);


//启动UR5
bool OpenUR5()
{
	//与UR5服务器通信打开
	if (!m_pUR5->Init(IPADDR, PORT1, PORT2))
	{
		printf("failed to open UR5 connection: '%s'\n", m_pUR5->GetLastError().c_str());
		return false;
	}
	//等待通信建立成功
	while (m_pUR5->GetStatus() != UR5SocketCom_RobotIdle)
	{
		Sleep(8);
	}
	//设置UR5运动记录输出文档
	m_pUR5->SetOutputFile();
	//启动UR5机器人
	if (!m_pUR5->EnableRobot())
	{
		printf("enable UR5 failed: '%s'\n", m_pUR5->GetLastError().c_str());
		return false;
	}
	//设置TCP
	Pose3D	define_TCP;
	define_TCP.Set(TCPPOSE);
	if (!m_pUR5->SetTCPTransformation(define_TCP))
	{
		printf("define TCP failed: '%s'\n", m_pUR5->GetLastError().c_str());
		return false;
	}
	//设置负载重量
	if (!m_pUR5->SetPayload(PAYLOAD))
	{
		printf("define payload failed: '%s'\n", m_pUR5->GetLastError().c_str());
		return false;
	}
	return true;
}
//初始化Kinect和图像矩阵
bool Init()
{
	count = 0;
	m_pKinect = new KinectDriver;
	
	m_pGraph = new Graphics;
	m_pGraph->DepthImg = new cv::Mat(DEPTHHEIGHT, DEPTHWIDTH, DEPTHFORMAT);
	m_pGraph->CloudPointsImg = new cv::Mat;
	m_pGraph->ColorImg = new cv::Mat(COLORHEIGHT, COLORWIDTH, COLORFORMAT);
	return true;
}

bool Uninit()
{
	delete m_pGraph->ColorImg;
	delete m_pGraph->DepthImg;
	delete m_pGraph;
	delete m_pKinect;
	return true;
}
//获取图像中心位置（实际未使用）
bool GetCenterPos(Pose &pos)
{
	Pose posCircle,posLine;
	if (m_pKinect->GetKinectImage(m_pGraph) != GT_RES_OK)
	{
		printf("Get Kinect Image failed\n");
		return false;
	}
	if (!GetCirclePos(*(m_pGraph->ColorImg), posCircle))
	{
		printf("GetCircleCenter failed!\n");
		return false;
	}
	if (!GetLinePos(*(m_pGraph->ColorImg), posLine))
	{
		printf("GetLineCenter failed!\n");
		return false;
	}
	posLine.GetAverage(posCircle);
	pos = posLine;
	return true;
}
//获取机器人当前TCP位置
bool GetRobotPos(Pose &pos)
{
	Pose3D pos3;
	pos3 = m_pUR5->GetTCPPose();
	pos.x = pos3.x;
	pos.y = pos3.y;
	pos.z = pos3.z;
	return true;
}
//在图片上绘制一个圆形
bool DrawCircle(cv::Mat Image, Pose &pos)
{
	if (!(Image.data))
	{
		return false;
	}
	cv::Point center(cvRound(pos.x), cvRound(pos.y));
	int radius = cvRound(pos.z);
	circle(Image, center, 3, cv::Scalar(0, 255, 0), 1, 8, 0);
	circle(Image, center, radius, cv::Scalar(0, 255, 0), 1, 8, 0);
	cv::imwrite("image" + std::to_string(count) + ".jpg", Image);
	count++;
	return true;
}
//从灰度图中识别圆形
bool GetCirclePos(cv::Mat Image,Pose &posCircle)
{
	if (!(Image.data))
	{
		return false;
	}
	Pose sum, pos;
	int CircleCnt = 0;
	for (size_t i = _MIN_RADIUS_; i < _MAX_RADIUS_; i += _DETAL_)
	{
		cv::Mat ImageGau;
		cv::GaussianBlur(Image, ImageGau, cv::Size(7, 7), 2, 2);
		std::vector<cv::Vec3f> circles;
		HoughCircles(ImageGau, circles, CV_HOUGH_GRADIENT, 1.5, 10, 200, 150, i, i+_DETAL_);
#ifdef _DEBUG_PRINT_
		//std::cout << i << "~" << i + _DETAL_ << ":";
#endif
		if (circles.size() == 0)
		{
#ifdef _DEBUG_PRINT_
			//std::cout << "Get not any circle" << std::endl;
#endif
			continue;
		}
		else if (circles.size() == 1)
		{
			pos.x = circles[0][0];
			pos.y = circles[0][1];
			pos.z = circles[0][2];
#ifdef _DEBUG_PRINT_
			//std::cout << "radius " << pos.z << std::endl;
#endif

		}
		else if (circles.size() > 1)
		{
#ifdef _DEBUG_PRINT_
			//std::cout << "Get more than one circles:" << circles.size() << std::endl;
#endif
			float CircleCnt = circles.size();
			float x = 0, y = 0, radius = 0;;
			for (size_t i = 0; i < CircleCnt; i++)
			{
				x += circles[i][0];
				y += circles[i][1];
				radius += circles[i][2];
#ifdef _DEBUG_PRINT_
				//std::cout << "radius " << circles[i][2] << " ";
#endif
			}
			std::cout << std::endl;
			pos.x = x / CircleCnt;
			pos.y = y / CircleCnt;
			pos.z = radius / CircleCnt;
		}
#ifdef _DEBUG_PRINT_
		DrawCircle(ImageGau, pos);
#endif
		sum = sum + pos;
		CircleCnt++;
	}
	if (CircleCnt == 0)
	{
		std::cout << "Get not any circle!" << std::endl;
		return false;
	}
	posCircle = sum / CircleCnt;
	DrawCircle(Image, posCircle);
	std::cout << "Get " << CircleCnt << " circles" << std::endl;
	return true;
}
//从图中识别直线（实际未使用）
bool GetLinePos(cv::Mat Image, Pose &pos)
{
	if (!(Image.data))
	{
		return false;
	}
	pos.x = 0;
	pos.y = 0;
	cv::Mat binary;
	Canny(Image, binary, 50, 100, 3);
	unsigned int count = 0;
	std::vector<cv::Vec2f> lines;
	HoughLines(binary, lines, 1, CV_PI / 180, 150, 0, 0);
	for (size_t i = 0; i < lines.size(); i++)
	{
		for (size_t j = i + 1; j < lines.size(); j++)
		{
			if (fabs(lines[i][1] - lines[j][1]) > 1e-3)
			{
				Pose tmp = CrossPoint(lines[i], lines[j]);
				pos.x += tmp.x;
				pos.y += tmp.y;
				count++;
			}
		}
	}
	std::cout << "count" << count << std::endl;
	pos.x /= count*1.0;
	pos.y /= count*1.0;
	return true;
}
//获取两直线的交叉点位置（实际未使用）
Pose CrossPoint(cv::Vec2f line1, cv::Vec2f line2)
{
	float rho1 = line1[0], rho2 = line2[0];
	float theta1 = line1[1], theta2 = line2[1];
	Pose pos;
	pos.y = (rho1*cos(theta1) + rho1*sin(theta1)*tan(theta1) - rho2*cos(theta2) - rho2*sin(theta2)*tan(theta2)) / (tan(theta1 - theta2));
	pos.x = rho1*sin(theta1)*tan(theta1)-pos.y*tan(theta1)+rho1*cos(theta1);
	return pos;
}
//将UINT16格式深度图转换为BYTE（仅保留距离低8位的细节信息）
bool GetBYTEformat(cv::Mat *DepthImg, cv::Mat *OutImg)
{
	int nWidth = DepthImg->size.p[1];
	int nHeight = DepthImg->size.p[0];
	for (size_t i = 0; i < nHeight; i++)
	{
		for (size_t j = 0; j < nWidth; j++)
		{
			OutImg->at<uchar>(i, j) = (uchar)(DepthImg->at<UINT16>(i, j) % 256);
		}
	}
	return true;
}
//判断相机坐标是否超出范围
bool IsInLimit(CameraSpacePoint pos)
{
	if (pos.X > 2.0 || pos.X < -2.0)
		return false;
	if (pos.Y > 2.0 || pos.Y < -2.0)
		return false;
	if (pos.Z > 2.0 || pos.Z < 0.0)
		return false;
	return true;
}
//阻塞方式运动到指定点
bool MovetoPos(Pose3D URpos)
{
	UR5SocketCom_Status status = m_pUR5->GetStatus();
	if (status != UR5SocketCom_RobotIdle && status != UR5SocketCom_Success)
	{
		printf("UR5 Robot's status is %u.\n", status);
		return false;
	}
	if (!(m_pUR5->MoveTCP(URpos)))
	{
		printf("MoveTCP failed!\n");
		return false;
	}
	Sleep(50);	//Wait until Robot change to Move station
	while ((status = m_pUR5->GetStatus()) != UR5SocketCom_RobotIdle)
	{
		Sleep(8);
	}
	return true;
}
//回复到指定位置
bool ResetPos()
{
	Pose3D pos;
	pos.Set(POSE1);
	return MovetoPos(pos);
}
//测试存储多幅图片，深度和彩色
bool TestSaveMoreThanOneImage()
{
	GT_RES res;

	//Initialize the Kinect driver
	m_pKinect = new KinectDriver;
	//Initialize the struct graph to store image
	m_pGraph = new Graphics;
	m_pGraph->DepthImg = new cv::Mat(DEPTHHEIGHT, DEPTHWIDTH, DEPTHFORMAT);
	m_pGraph->ColorImg = new cv::Mat(COLORHEIGHT, COLORWIDTH, COLORFORMAT);
	m_pGraph->CloudPointsImg = new cv::Mat;
	UINT16 *depthimg = new UINT16[DEPTHHEIGHT*DEPTHWIDTH];
	cv::Mat *depthmat = new cv::Mat(DEPTHHEIGHT, DEPTHWIDTH, DEPTHFORMAT);
	cv::Mat DepthInBYTEImg1(DEPTHHEIGHT, DEPTHWIDTH, COLORFORMAT);
	//Get the Kinect image(Color & depth & depth in color frame)
	Sleep(3000);													//Kinect initializing need time, so wait 2sencods to make sure Kinect is ready
	m_pKinect->GetDepthImage(depthimg);
	m_pKinect->DepthConvertMat(depthimg,DEPTHWIDTH,DEPTHHEIGHT,depthmat);
	GetBYTEformat(depthmat, &DepthInBYTEImg1);
	cv::imwrite("Depth1.jpg", *depthmat);
	cv::imwrite("DepthInBYTEImg1.jpg", DepthInBYTEImg1);
	for (int i = 0; i < 1; i++)
	{
		res = GT_RES_ERROR;
		while (res != GT_RES_OK)
		{
			res = m_pKinect->GetKinectImage(m_pGraph);
			if (res != GT_RES_OK)
			{
				//printf("Get Kinect image error:%02x\n", res);
			}
		}

		std::string prefix1("ColorImg");
		std::string prefix2("DepthImg");
		std::string prefix3("DepthInBYTEImg");
		cv::Mat DepthInBYTEImg(DEPTHHEIGHT, DEPTHWIDTH, COLORFORMAT);
		GetBYTEformat(m_pGraph->DepthImg, &DepthInBYTEImg);

#ifdef _DEBUG_PRINT_
		std::cout << "Saving image:" << prefix1 << i << ".jpg," << prefix2 << i << ".jpg," << prefix3 << i << ".jpg" << std::endl;
#endif
		bool bl1, bl2, bl3;
		bl1 = cv::imwrite(prefix1 + std::to_string(i) + ".jpg", *(m_pGraph->ColorImg));
		bl2 = cv::imwrite(prefix2 + std::to_string(i) + ".jpg", *(m_pGraph->DepthImg));
		bl3 = cv::imwrite(prefix3 + std::to_string(i) + ".jpg", DepthInBYTEImg);

		if (!(bl1&&bl2&&bl3))
		{
			std::cout << "save image failed" << std::endl;
			return false;
		}
	}
	std::cout << "Kinect can save more than one image" << std::endl;

	return true;
}
//测试进行Robot和Camera坐标系校准
bool TestCalibration()
{
	GT_RES res;
	Pose pos;
	Init();
	printf("init ok\n");
	std::ofstream fp;
	fp.open("coordinate.txt", std::ios::out);
	Sleep(3000);
	for (size_t i = 0; i < 10; i++)
	{

		std::cin.get();
		CameraSpacePoint CameraposSum;
		CameraposSum.X = 0;
		CameraposSum.Y = 0;
		CameraposSum.Z = 0;
		for (size_t j = 0; j < 5; j++)													//获取多幅图片5，减少彩色图片和深度图片的误差
		{
			res = m_pKinect->GetKinectImage(m_pGraph);
			if (res != GT_RES_OK)
			{
				std::cout << "Get image error\n" << std::endl;
				continue;
			}
			if (!GetCirclePos(*(m_pGraph->ColorImg), pos))
			{
				std::cout << "get image center failed" << std::endl;
				continue;
			}
			std::cout << std::setprecision(7) << "Colorpos: " << pos.x << " " << pos.y << " " << pos.z << " ";

			std::vector<ColorSpacePoint> v_Colorpos;
			std::vector<CameraSpacePoint> v_Camerapos;
			for (size_t i = 0; i < 5; i++)
			{
				for (size_t j = 0; j < 5; j++)
				{
					ColorSpacePoint pos_tmp;
					pos_tmp.X = pos.x + i - 2;
					pos_tmp.Y = pos.y + j - 2;
					v_Colorpos.push_back(pos_tmp);
				}
			}
			res = m_pKinect->Colorpos2Camerapos(v_Colorpos, v_Camerapos);
			if (res != GT_RES_OK)
			{
				std::cout << "Coordiante color to camera failed with error:" << res << std::endl;
				continue;
			}
			CameraSpacePoint Camerapos;
			Camerapos.X = 0;
			Camerapos.Y = 0;
			Camerapos.Z = 0;
			int sizeCnt = 0;
			for (size_t i = 0; i < v_Camerapos.size(); i++)
			{
				if (IsInLimit(v_Camerapos[i]))
				{
					Camerapos.X += v_Camerapos[i].X;
					Camerapos.Y += v_Camerapos[i].Y;
					Camerapos.Z += v_Camerapos[i].Z;
					sizeCnt++;
				}
			}
			Camerapos.X /= sizeCnt;
			Camerapos.Y /= sizeCnt;
			Camerapos.Z /= sizeCnt;
			CameraposSum.X += Camerapos.X;
			CameraposSum.Y += Camerapos.Y;
			CameraposSum.Z += Camerapos.Z; 
			std::cout << std::setprecision(7) << sizeCnt << "\t" << Camerapos.X << "\t" << Camerapos.Y << "\t" << Camerapos.Z << "\n";
		}
		CameraposSum.X /= 5;
		CameraposSum.Y /= 5;
		CameraposSum.Z /= 5;
		fp << std::setprecision(7) << CameraposSum.X << "\t" << CameraposSum.Y << "\t" << CameraposSum.Z << "\t";
		std::cout << std::setprecision(7) << 5 << "\n" << CameraposSum.X << "\t" << CameraposSum.Y << "\t" << CameraposSum.Z << "\t";


		std::cin.get();

		m_pUR5 = new UR5SocketCom;
		GT_RES res = OpenUR5();
		if (res != GT_RES_OK)
		{
			std::cout << "open UR5 failed with error:" << res << std::endl;
			delete m_pUR5;
			continue;
		}
		if (!GetRobotPos(pos))
		{
			std::cout << "get robot pos failed" << std::endl;
			delete m_pUR5;
			continue;
		}
		fp << std::setprecision(7) << pos.x << "\t" << pos.y << "\t" << pos.z << std::endl;
		std::cout << std::setprecision(7) << pos.x << "\t" << pos.y << "\t" << pos.z << std::endl;
		if (!m_pUR5->DisableRobot())
		{
			std::cout << "disable UR5 failed: '%s'" << m_pUR5->GetLastError().c_str() << std::endl;
			delete m_pUR5;
			continue;
		}
		delete m_pUR5;

	}
	fp.close();
	Uninit();
	return true;
}
//测试机器人主动移动，进行坐标系的校准
bool TestMoveCalibration()
{
	GT_RES res;
	Init();
	std::ofstream fp;
	srand((unsigned int)(time(NULL)));
	fp.open("coordinate.txt", std::ios::app);
	Sleep(3000);
	m_pUR5 = new UR5SocketCom;
	while ((res = OpenUR5()) != GT_RES_OK)
	{
		std::cout << "open UR5 failed with error:" << res << std::endl;
	}
	printf("init ok\n");
	for (size_t i = 0; i < 10; i++)
	{
		Pose pos_r,pos_c;
		float x, y, z;
		x = rand() / (double)(RAND_MAX) * 0.1;
		y = -1 * (rand() / (double)(RAND_MAX) * 0.04 + 0.46);
		z = rand() / (double)(RAND_MAX) * 0.21 + 0.19;
		printf("move to pos:%f %f %f\n", x, y, z);
		if (!(m_pUR5->MoveTCP(x,y,z,PI/2,PI/2-0.04,0)))
		{
			printf("move tcp failed!\n");
			continue;
		}
		std::wcout << "Wait to Get robot pos....." << std::endl;
		Sleep(1000);
		if (!GetRobotPos(pos_r))
		{
			std::cout << "get robot pos failed" << std::endl;
			continue;
		}
		std::cout << "error:" << std::setprecision(7) << pos_r.x - x << "\t" << pos_r.y - y << "\t" << pos_r.z - z << "\n";
		std::cout << std::setprecision(7) << pos_r.x << "\t" << pos_r.y << "\t" << pos_r.z << "\n";

		std::cin.get();
		CameraSpacePoint CameraposSum;
		CameraSpacePoint CameraposMax;
		CameraSpacePoint CameraposMin;
		CameraSpacePoint CameraposSquareSum;
		CameraSpacePoint Camerapos_v[5];
		CameraposSum.X = 0;
		CameraposSum.Y = 0;
		CameraposSum.Z = 0;
		CameraposMax.X = -100;
		CameraposMax.Y = -100;
		CameraposMax.Z = -100;
		CameraposMin.X = 100.0;
		CameraposMin.Y = 100.0;
		CameraposMin.Z = 100.0;
		CameraposSquareSum.X = 0;
		CameraposSquareSum.Y = 0;
		CameraposSquareSum.Z = 0;
		for (size_t j = 0; j < 5; j++)													//获取多幅图片5，减少彩色图片和深度图片的误差
		{
			res = m_pKinect->GetKinectImage(m_pGraph);
			if (res != GT_RES_OK)
			{
				std::cout << "Get image error\n" << std::endl;
				continue;
			}
			if (!GetCirclePos(*(m_pGraph->ColorImg), pos_c))
			{
				std::cout << "get image center failed" << std::endl;
				continue;
			}
			std::cout << std::setprecision(7) << "Colorpos: " << pos_c.x << "\t" << pos_c.y << "\t" << pos_c.z << "\t";

			std::vector<ColorSpacePoint> v_Colorpos;
			std::vector<CameraSpacePoint> v_Camerapos;
			for (size_t i = 0; i < 5; i++)
			{
				for (size_t j = 0; j < 5; j++)
				{
					ColorSpacePoint pos_tmp;
					pos_tmp.X = pos_c.x + i - 2;
					pos_tmp.Y = pos_c.y + j - 2;
					v_Colorpos.push_back(pos_tmp);
				}
			}
			res = m_pKinect->Colorpos2Camerapos(v_Colorpos, v_Camerapos);
			if (res != GT_RES_OK)
			{
				std::cout << "Coordiante color to camera failed with error:" << res << std::endl;
				continue;
			}
			Camerapos_v[j].X = 0;
			Camerapos_v[j].Y = 0;
			Camerapos_v[j].Z = 0;
			int sizeCnt = 0;
			for (size_t k = 0; k < v_Camerapos.size(); k++)
			{
				if (IsInLimit(v_Camerapos[k]))
				{
					Camerapos_v[j].X += v_Camerapos[k].X;
					Camerapos_v[j].Y += v_Camerapos[k].Y;
					Camerapos_v[j].Z += v_Camerapos[k].Z;
					sizeCnt++;
				}
			}
			Camerapos_v[j].X /= sizeCnt;
			Camerapos_v[j].Y /= sizeCnt;
			Camerapos_v[j].Z /= sizeCnt;
			std::cout << std::setprecision(7) << sizeCnt << "\t" << Camerapos_v[j].X << "\t" << Camerapos_v[j].Y << "\t" << Camerapos_v[j].Z << "\n";
		}
		for (size_t j = 0; j < 5; j++)
		{
			CameraposMax.X = max(CameraposMax.X, Camerapos_v[j].X);
			CameraposMax.Y = max(CameraposMax.Y, Camerapos_v[j].Y);
			CameraposMax.Z = max(CameraposMax.Z, Camerapos_v[j].Z);
			CameraposMin.X = min(CameraposMin.X, Camerapos_v[j].X);
			CameraposMin.Y = min(CameraposMin.Y, Camerapos_v[j].Y);
			CameraposMin.Z = min(CameraposMin.Z, Camerapos_v[j].Z);
			CameraposSum.X += Camerapos_v[j].X;
			CameraposSum.Y += Camerapos_v[j].Y;
			CameraposSum.Z += Camerapos_v[j].Z;
			CameraposSquareSum.X += Camerapos_v[j].X*Camerapos_v[j].X;
			CameraposSquareSum.Y += Camerapos_v[j].Y*Camerapos_v[j].Y;
			CameraposSquareSum.Z += Camerapos_v[j].Z*Camerapos_v[j].Z;
			
		}
		CameraposSum.X /= 5;
		CameraposSum.Y /= 5;
		CameraposSum.Z /= 5;
		CameraposSquareSum.X = CameraposSquareSum.X - CameraposSum.X*CameraposSum.X;
		std::cout << std::setprecision(7) << CameraposSum.X << "\t" << CameraposSum.Y << "\t" << CameraposSum.Z << std::endl;
		std::cout << std::setprecision(7) << CameraposSquareSum.X << "\t" << CameraposSquareSum.Y << "\t" << CameraposSquareSum.Z << std::endl;
		std::cout << std::setprecision(7) << CameraposMax.X-CameraposMin.X << "\t" << CameraposMax.Y - CameraposMin.Y << "\t" << CameraposMax.Z - CameraposMin.Z << std::endl;
		std::cout << "record this data?(y or n)" << std::endl;
		if (std::cin.get() == 'y')
		{
			fp << std::setprecision(7) << pos_r.x << "\t" << pos_r.y << "\t" << pos_r.z << "\t";
			fp << std::setprecision(7) << CameraposSum.X << "\t" << CameraposSum.Y << "\t" << CameraposSum.Z << std::endl;
		}
	}
	fp.close();
	Uninit();
	if (!m_pUR5->DisableRobot())
	{
		std::cout << "disable UR5 failed: '%s'" << m_pUR5->GetLastError().c_str() << std::endl;
	}
	delete m_pUR5;
	return true;
}
//测试显示图片窗口
bool TestShowImage()
{
	Init();
	printf("Init Ok\n");
	Sleep(2000);

	m_pKinect->GetKinectImage(m_pGraph);

	Pose pos;
	pos.x = (COLORSPACELEFT + COLORSPACERIGHT) / 2.0;
	pos.y = (COLORSPACEUP + COLORSPACEDOWN) / 2.0;
	cv::Point center(cvRound(pos.x), cvRound(pos.y));
	//cv::circle(*(m_pGraph->ColorImg), center, 13, cv::Scalar(0, 255, 0), 3, 8, 0);
	cv::Rect rect1(pos.x - (COLORSPACERIGHT - COLORSPACELEFT) / 2.0, pos.y - (COLORSPACEDOWN - COLORSPACEUP) / 2.0, (COLORSPACERIGHT - COLORSPACELEFT), (COLORSPACEDOWN - COLORSPACEUP));
	cv::namedWindow("color", CV_WINDOW_NORMAL);
	cv::imshow("color", (*(m_pGraph->ColorImg))(rect1));
	cv::imwrite("color.jpg", (*(m_pGraph->ColorImg))(rect1));

	float Dx, Dy;
	m_pKinect->Colorpos2Depthpos(pos.x, pos.y, Dx, Dy);
	cv::Point center2(cvRound(Dx), cvRound(Dy));
	//cv::circle(*(m_pGraph->DepthImg), center2, 13, cv::Scalar(0, 255, 0), 3, 8, 0);
	cv::Rect rect2(Dx - (COLORSPACERIGHT - COLORSPACELEFT) / 4.0, Dy - (COLORSPACEDOWN - COLORSPACEUP) / 4.0, (COLORSPACERIGHT - COLORSPACELEFT) / 2.0, (COLORSPACEDOWN - COLORSPACEUP) / 2.0);
	cv::Mat img(m_pGraph->DepthImg->rows, m_pGraph->DepthImg->cols, COLORFORMAT);
	GetBYTEformat((m_pGraph->DepthImg),&img);
	cv::namedWindow("depth", CV_WINDOW_NORMAL);
	cv::imshow("depth", img(rect2));
	cv::imwrite("depth.jpg", img(rect2));

	cv::namedWindow("CloudPoints", CV_WINDOW_AUTOSIZE);
	cv::imshow("CloudPoints", (*m_pGraph->CloudPointsImg));
	cv::imwrite("CloudPoints.jpg", (*m_pGraph->CloudPointsImg));

	cv::waitKey(0);


	return true;
}
//测试转换矩阵精度
bool TestVerifyTransMat()
{
	Pose pos;
	Init();
	m_pUR5 = new UR5SocketCom;
	GT_RES res;
	if (!OpenUR5())
	{
		std::cout << "open UR5 failed with error:" << res << std::endl;
		delete m_pUR5;
		return false;
	}

	printf("init ok\n");
	Sleep(2000);
	while (1)
	{
		//input 'Enter' :move to next step, input Space and 'Enter': End the program
		if (std::cin.get() == ' ')
			return true;
		
		res = m_pKinect->GetKinectImage(m_pGraph);
		if (res != GT_RES_OK)
		{
			std::cout << "Get image error\n" << std::endl;
			continue;
		}
		if (!GetCirclePos(*(m_pGraph->ColorImg), pos))
		{
			std::cout << "get image center failed" << std::endl;
			continue;
		}
		std::cout << "Get pos:" << pos.x << " " << pos.y << std::endl;
		GraspPose grasppos;
		grasppos.x = pos.x;
		grasppos.y = pos.y;
		grasppos.theta = 0;
		Pose3D URpos;
		
		res = m_pKinect->ColorDepth2Robot(grasppos, URpos);
		if (res != GT_RES_OK)
		{
			std::cout << "Grasppos transform to URpos failed " << std::endl;
			continue;
		}
		std::cout << "Move to " << URpos.ToString() << std::endl;
		if (!MovetoPos(URpos))
		{
			std::cout << "Move to pos failed" << std::endl;
			continue;
		}

		if (std::cin.get() == ' ')
			return true;

		if (!ResetPos())
		{
			std::cout << "Reset UR pos failed" << std::endl;
			continue;
		}
	}
	return true;
}
//测试UR5本身的精度
bool TestErrorUR5(int Num)
{
	m_pUR5 = new UR5SocketCom;
	if (!OpenUR5())
	{
		std::cout << "open UR5 failed" << std::endl;
		delete m_pUR5;
		return false;
	}
	printf("init ok\n");
	Pose3D *error = new Pose3D[Num];
	Pose3D pos,pos2,sum,max;
	max.Set(0, 0, 0, 0, 0, 0);

	pos2 = m_pUR5->GetTCPPose();

	for (size_t i = 0; i < Num; i++)
	{
		pos.Set(POSE2);
		/*if (!MovetoPos(pos))
		{
			std::cout << "Moveto pos failed" << std::endl;
			return false;
		}*/
		Sleep(500);
		pos = pos2;
		pos2 = m_pUR5->GetTCPPose();
		error[i].x = pos2.x - pos.x;
		error[i].y = pos2.y - pos.y;
		error[i].z = pos2.z - pos.z;
		error[i].Rx = pos2.Rx - pos.Rx;
		error[i].Ry = pos2.Ry - pos.Ry;
		error[i].Rz = pos2.Rz - pos.Rz;
		std::cout << error[i].ToString() << std::endl;
		sum.x += fabs(error[i].x);
		sum.y += fabs(error[i].y);
		sum.z += fabs(error[i].z);
		sum.Rx += fabs(error[i].Rx);
		sum.Ry += fabs(error[i].Ry);
		sum.Rz += fabs(error[i].Rz);
		max.x = MAX(max.x, fabs(error[i].x));
		max.y = MAX(max.y, fabs(error[i].y));
		max.z = MAX(max.z, fabs(error[i].z));
		max.Rx = MAX(max.Rx, fabs(error[i].Rx));
		max.Ry = MAX(max.Ry, fabs(error[i].Ry));
		max.Rz = MAX(max.Rz, fabs(error[i].Rz));
		//ResetPos();
	}
	sum.x /= Num;
	sum.y /= Num;
	sum.z /= Num;
	sum.Rx /= Num;
	sum.Ry /= Num;
	sum.Rz /= Num;
	std::cout << "average:" << sum.ToString() << std::endl;
	std::cout << "max:" << max.ToString() << std::endl;
	return true;
}

int main()
{
	//TestShowImage();
	TestSaveMoreThanOneImage();
	//TestMoveCalibration();
	//TestVerifyTransMat();
    return 0;
}

