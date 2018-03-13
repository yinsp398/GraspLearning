// CoordinateCalibration.cpp : 定义控制台应用程序的入口点。
//
#include <Common.h>
#include <opencv2\opencv.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include <KinectDriver.h>
#include <UR5SocketCom.h>
#include <UR5SocketComCodes.h>
#include <utils.h>
#include <fstream>

#define _DEBUG_PRINT_

struct Pose {
	float x;
	float y;
	float z;
	void GetAverage(Pose pos)
	{
		x = (pos.x + x) / 2;
		y = (pos.y + y) / 2;
		z = (pos.z + z) / 2;
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
	define_TCP.Set(0.0, 0.0, 0.2, 0.0, 0.0, 0.0);
	if (!m_pUR5->SetTCPTransformation(define_TCP))
	{
		printf("define TCP failed: '%s'\n", m_pUR5->GetLastError().c_str());
		return false;
	}
	//设置负载重量
	if (!m_pUR5->SetPayload(1.0, 0.0, 0.0, 0.05))
	{
		printf("define payload failed: '%s'\n", m_pUR5->GetLastError().c_str());
		return false;
	}
	return true;
}

bool Init()
{
	GT_RES res;
	count = 0;
	m_pKinect = new KinectDriver;
	res = m_pKinect->OpenKinect();
	if (res != GT_RES_OK)
	{
		printf("open kinect failed with error:%02x\n", res);
		return false;
	}
	
	
	m_pGraph = new Graphics;
	m_pGraph->DepthImg = new cv::Mat(DEPTHHEIGHT, DEPTHWIDTH, DEPTHFORMAT);
	m_pGraph->ColorImg = new cv::Mat(COLORHEIGHT, COLORWIDTH, COLORFORMAT);
	m_pGraph->DepthInColorImg = new cv::Mat(COLORHEIGHT, COLORWIDTH, DEPTHFORMAT);
	return true;
}

bool Uninit()
{
	m_pKinect->CloseKinect();
	m_pUR5->DisableRobot();
	delete m_pGraph->ColorImg;
	delete m_pGraph->DepthImg;
	delete m_pGraph->DepthInColorImg;
	delete m_pGraph;
	delete m_pKinect;
	delete m_pUR5;
	return true;
}

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
	pos.z = m_pGraph->DepthInColorImg->at<UINT16>(pos.x, pos.y);
	return true;
}

bool GetRobotPos(Pose &pos)
{
	Pose3D pos3;
	pos3 = m_pUR5->GetTCPPose();
	pos.x = pos3.x;
	pos.y = pos3.y;
	pos.z = pos3.z;
	return true;
}

bool GetCirclePos(cv::Mat Image,Pose &pos)
{
	if (!(Image.data))
	{
		return false;
	}
	cv::GaussianBlur(Image, Image, cv::Size(7, 7), 2, 2);
	std::vector<cv::Vec3f> circles;
	HoughCircles(Image, circles, CV_HOUGH_GRADIENT, 1.5, 10, 200, 200, 100, 130);

#ifdef _DEBUG_PRINT_
	for (size_t i = 0; i < circles.size(); i++)
	{
		cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
		int radius = cvRound(circles[i][2]);
		circle(*(m_pGraph->ColorImg), center, 3, cv::Scalar(0, 255, 0), -1, 8, 0);
		circle(*(m_pGraph->ColorImg), center, radius, cv::Scalar(0, 255, 0), 6, 8, 0);
		std::cout << "radius" << radius << std::endl;
		cv::imwrite("image" + std::to_string(count) + ".jpg", *(m_pGraph->ColorImg));
		count++;
	}

#endif
	if (circles.size() == 0)
	{
		printf("Get not any circle\n");
		return false;
	}
	else if (circles.size() == 1)
	{
		pos.x = circles[0][0];
		pos.y = circles[0][1];

		return true;
	}
	else if (circles.size() > 1)
	{
		printf("Get more than one circles\n");
		return false;
	}
}

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

Pose CrossPoint(cv::Vec2f line1, cv::Vec2f line2)
{
	float rho1 = line1[0], rho2 = line2[0];
	float theta1 = line1[1], theta2 = line2[1];
	Pose pos;
	pos.y = (rho1*cos(theta1) + rho1*sin(theta1)*tan(theta1) - rho2*cos(theta2) - rho2*sin(theta2)*tan(theta2)) / (tan(theta1 - theta2));
	pos.x = rho1*sin(theta1)*tan(theta1)-pos.y*tan(theta1)+rho1*cos(theta1);
	return pos;
}

bool TestKinect()
{
	GT_RES res;

	//Initialize the Kinect driver
	m_pKinect = new KinectDriver;
	res = m_pKinect->OpenKinect();
	if (res != GT_RES_OK)
	{
		printf("open kinect failed with error:%02x\n", res);
		return false;
	}
	//Initialize the struct graph to store image
	m_pGraph = new Graphics;
	m_pGraph->DepthImg = new cv::Mat(DEPTHHEIGHT, DEPTHWIDTH, DEPTHFORMAT);
	m_pGraph->ColorImg = new cv::Mat(COLORHEIGHT, COLORWIDTH, COLORFORMAT);
	m_pGraph->DepthInColorImg = new cv::Mat(COLORHEIGHT, COLORWIDTH, DEPTHFORMAT);
	//Get the Kinect image(Color & depth & depth in color frame)
					
		Sleep(2000);													//Kinect initializing need time, so wait 2sencods to make sure Kinect is ready													
	for (int i = 0; i < 10; i++)
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
		std::string prefix("ColorImg");
		
#ifdef _DEBUG_PRINT_
		std::cout << "Saving image:" + prefix + std::to_string(i) + ".jpg" << std::endl;
#endif
		bool bl = cv::imwrite(prefix + std::to_string(i) + ".jpg", *(m_pGraph->ColorImg));
		if (!bl)
		{
			std::cout << "save image failed" << std::endl;
			return false;
		}
	}
	std::cout << "Kinect can save more than one image" << std::endl;

	return true;
}

int main()
{
#if 1
	Pose pos;
	Init();
	printf("init ok\n");
	std::ofstream fp;
	fp.open("coordinate.txt", std::ios::out);
	Sleep(2000);
	for (size_t i = 0; i < 10; i++)
	{
		//while (std::cin.get() != '\n');
		std::cin.get();

		if (m_pKinect->GetKinectImage(m_pGraph)!=GT_RES_OK)
		{
			printf("Get image error\n");
			return 1;
		}
		if (!GetCirclePos(*(m_pGraph->ColorImg),pos))
		{
			std::cout << "get image center failed" << std::endl;
			return 1;
		}
		pos.z = m_pGraph->DepthInColorImg->at<UINT16>(int(pos.y + 0.5), int(pos.x + 0.5));
		fp << pos.x << " " << pos.y << " " << pos.z<<"\t";
		std::cout << pos.x << " " << pos.y << " " << pos.z << "\t";

		std::cin.get();
		//while (std::cin.get() != '\n');

		m_pUR5 = new UR5SocketCom;
		GT_RES res = OpenUR5();
		if (res != GT_RES_OK)
		{
			printf("open UR5 failed with error:%02x\n", res);
			return false;
		}
		if (!GetRobotPos(pos))
		{
			std::cout << "get robot pos failed" << std::endl;
			return 1;
		}
		fp << pos.x << " " << pos.y << " " << pos.z << std::endl;
		std::cout << pos.x << " " << pos.y << " " << pos.z << std::endl;
		if (!m_pUR5->DisableRobot())
		{
			printf("disable UR5 failed: '%s'\n", m_pUR5->GetLastError().c_str());
			return 1;
		}
		delete m_pUR5;

	}
	fp.close();
	Uninit();
#else
	Init();
	printf("Init Ok\n");
	Sleep(2000);
	m_pKinect->GetKinectImage(m_pGraph);
	/*cv::Mat binary;
	Canny(*(m_pGraph->ColorImg), binary, 50, 100, 3);
	
	std::vector<cv::Vec2f> lines;
	HoughLines(binary, lines, 1, CV_PI / 180, 200, 0, 0);
	std::cout << "line number:" << lines.size() << std::endl;
	for (size_t i = 0; i < lines.size(); i++)
	{
		float rho = lines[i][0];
		float theta = lines[i][1];
		double a = cos(theta), b = sin(theta);
		cv::Point pt1, pt2;

		pt1.x = cvRound(a*rho + 1000 * (-b));
		pt1.y = cvRound(b*rho + 1000 * (a));
		pt2.x = cvRound(a*rho + 1000 * (b));
		pt2.y = cvRound(b*rho + 1000 * (-a));


		line(*(m_pGraph->ColorImg), pt1, pt2, cv::Scalar(255, 0, 255), 1, 8);
		std::cout << rho << " " << theta << std::endl;
	}
	*/
	cv::Mat grayImage;
	cv::GaussianBlur(*(m_pGraph->ColorImg), grayImage, cv::Size(7, 7), 2, 2);
	std::vector<cv::Vec3f> circles;
	std::vector<cv::Vec3f> circle_tmp;
	HoughCircles(grayImage, circles, CV_HOUGH_GRADIENT, 1.5, 10, 200, 200, 100, 130);
	std::cout << circles.size() << std::endl;
	for (size_t i = 0; i < circles.size(); i++)
	{
		cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
		int radius = cvRound(circles[i][2]);
		circle(*(m_pGraph->ColorImg), center, 3, cv::Scalar(0, 255, 0), -1, 8, 0);
		circle(*(m_pGraph->ColorImg), center, radius, cv::Scalar(155, 50, 255), 5, 8, 0);
		std::cout << "radius" << radius << std::endl;
	}
	cv::namedWindow("test", CV_WINDOW_NORMAL);
	
	cv::imshow("test", *(m_pGraph->ColorImg));
	
	cv::waitKey(0);
	
#endif
    return 0;
}

