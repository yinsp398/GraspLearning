// CoordinateCalibration.cpp : �������̨Ӧ�ó������ڵ㡣
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

//#define _DEBUG_PRINT_
#define _MAX_RADIUS_		120
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
bool TestShowImage();
bool TestVerifyTransMat();
bool TestErrorUR5();

//����UR5
bool OpenUR5()
{
	//��UR5������ͨ�Ŵ�
	if (!m_pUR5->Init(IPADDR, PORT1, PORT2))
	{
		printf("failed to open UR5 connection: '%s'\n", m_pUR5->GetLastError().c_str());
		return false;
	}
	//�ȴ�ͨ�Ž����ɹ�
	while (m_pUR5->GetStatus() != UR5SocketCom_RobotIdle)
	{
		Sleep(8);
	}
	//����UR5�˶���¼����ĵ�
	m_pUR5->SetOutputFile();
	//����UR5������
	if (!m_pUR5->EnableRobot())
	{
		printf("enable UR5 failed: '%s'\n", m_pUR5->GetLastError().c_str());
		return false;
	}
	//����TCP
	Pose3D	define_TCP;
	define_TCP.Set(TCPPOSE);
	if (!m_pUR5->SetTCPTransformation(define_TCP))
	{
		printf("define TCP failed: '%s'\n", m_pUR5->GetLastError().c_str());
		return false;
	}
	//���ø�������
	if (!m_pUR5->SetPayload(PAYLOAD))
	{
		printf("define payload failed: '%s'\n", m_pUR5->GetLastError().c_str());
		return false;
	}
	return true;
}
//��ʼ��Kinect��ͼ�����
bool Init()
{
	count = 0;
	m_pKinect = new KinectDriver;
	
	m_pGraph = new Graphics;
	m_pGraph->DepthImg = new cv::Mat(DEPTHHEIGHT, DEPTHWIDTH, DEPTHFORMAT);
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
//��ȡͼ������λ�ã�ʵ��δʹ�ã�
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
//��ȡ�����˵�ǰTCPλ��
bool GetRobotPos(Pose &pos)
{
	Pose3D pos3;
	pos3 = m_pUR5->GetTCPPose();
	pos.x = pos3.x;
	pos.y = pos3.y;
	pos.z = pos3.z;
	return true;
}
//��ͼƬ�ϻ���һ��Բ��
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
//�ӻҶ�ͼ��ʶ��Բ��
bool GetCirclePos(cv::Mat Image,Pose &posCircle)
{
	if (!(Image.data))
	{
		return false;
	}
	Pose sum, pos;
	int CircleCnt = 0;
	for (size_t i = 0; i < _MAX_RADIUS_; i += _DETAL_)
	{
		cv::Mat ImageGau;
		cv::GaussianBlur(Image, ImageGau, cv::Size(7, 7), 2, 2);
		std::vector<cv::Vec3f> circles;
		HoughCircles(ImageGau, circles, CV_HOUGH_GRADIENT, 1.5, 10, 200, 150, i, i+_DETAL_);
#ifdef _DEBUG_PRINT_
		std::cout << i << "~" << i + _DETAL_ << ":";
#endif
		if (circles.size() == 0)
		{
#ifdef _DEBUG_PRINT_
			std::cout << "Get not any circle" << std::endl;
			continue;
#endif
		}
		else if (circles.size() == 1)
		{
			pos.x = circles[0][0];
			pos.y = circles[0][1];
			pos.z = circles[0][2];
#ifdef _DEBUG_PRINT_
			std::cout << "radius " << pos.z << std::endl;
#endif

		}
		else if (circles.size() > 1)
		{
#ifdef _DEBUG_PRINT_
			std::cout << "Get more than one circles:" << circles.size() << std::endl;
#endif
			float CircleCnt = circles.size();
			float x = 0, y = 0, radius = 0;;
			for (size_t i = 0; i < CircleCnt; i++)
			{
				x += circles[i][0];
				y += circles[i][1];
				radius += circles[i][2];
#ifdef _DEBUG_PRINT_
				std::cout << "radius " << circles[i][2] << " ";
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
	posCircle = sum / CircleCnt;
	DrawCircle(Image, posCircle);
	return true;
}
//��ͼ��ʶ��ֱ�ߣ�ʵ��δʹ�ã�
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
//��ȡ��ֱ�ߵĽ����λ�ã�ʵ��δʹ�ã�
Pose CrossPoint(cv::Vec2f line1, cv::Vec2f line2)
{
	float rho1 = line1[0], rho2 = line2[0];
	float theta1 = line1[1], theta2 = line2[1];
	Pose pos;
	pos.y = (rho1*cos(theta1) + rho1*sin(theta1)*tan(theta1) - rho2*cos(theta2) - rho2*sin(theta2)*tan(theta2)) / (tan(theta1 - theta2));
	pos.x = rho1*sin(theta1)*tan(theta1)-pos.y*tan(theta1)+rho1*cos(theta1);
	return pos;
}
//��UINT16��ʽ���ͼת��ΪBYTE�������������8λ��ϸ����Ϣ��
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
//�ж���������Ƿ񳬳���Χ
bool IsInLimit(CameraSpacePoint pos)
{
	if (pos.X > 2.0 || pos.X < -2.0)
		return false;
	if (pos.Y > 2.0 || pos.Y < -2.0)
		return false;
	if (pos.Z > 1.1 || pos.Z < 0.8)
		return false;
	return true;
}
//������ʽ�˶���ָ����
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
//�ظ���ָ��λ��
bool ResetPos()
{
	Pose3D pos;
	pos.Set(POSE1);
	return MovetoPos(pos);
}
//���Դ洢���ͼƬ����ȺͲ�ɫ
bool TestSaveMoreThanOneImage()
{
	GT_RES res;

	//Initialize the Kinect driver
	m_pKinect = new KinectDriver;
	//Initialize the struct graph to store image
	m_pGraph = new Graphics;
	m_pGraph->DepthImg = new cv::Mat(DEPTHHEIGHT, DEPTHWIDTH, DEPTHFORMAT);
	m_pGraph->ColorImg = new cv::Mat(COLORHEIGHT, COLORWIDTH, COLORFORMAT);
	//Get the Kinect image(Color & depth & depth in color frame)

	Sleep(3000);													//Kinect initializing need time, so wait 2sencods to make sure Kinect is ready													
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
//���Խ���Robot��Camera����ϵУ׼
bool TestCalibration()
{
	GT_RES res;
	Pose pos;
	Init();
	printf("init ok\n");
	std::ofstream fp;
	fp.open("coordinate.txt", std::ios::out);
	Sleep(2000);
	for (size_t i = 0; i < 10; i++)
	{

		std::cin.get();
		CameraSpacePoint CameraposSum;
		CameraposSum.X = 0;
		CameraposSum.Y = 0;
		CameraposSum.Z = 0;
		for (size_t j = 0; j < 5; j++)													//��ȡ���ͼƬ5�����ٲ�ɫͼƬ�����ͼƬ�����
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
			std::cout << "Colorpos: " << pos.x << " " << pos.y << " " << pos.z << " ";

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
			std::cout << sizeCnt << "\t" << Camerapos.X << "\t" << Camerapos.Y << "\t" << Camerapos.Z << "\n";
		}
		CameraposSum.X /= 5;
		CameraposSum.Y /= 5;
		CameraposSum.Z /= 5;
		fp << CameraposSum.X << "\t" << CameraposSum.Y << "\t" << CameraposSum.Z << "\t";
		std::cout << 5 << "\n" << CameraposSum.X << "\t" << CameraposSum.Y << "\t" << CameraposSum.Z << "\t";


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
		fp << pos.x << "\t" << pos.y << "\t" << pos.z << std::endl;
		std::cout << pos.x << "\t" << pos.y << "\t" << pos.z << std::endl;
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
//������ʾͼƬ����
bool TestShowImage()
{
	Init();
	printf("Init Ok\n");
	Sleep(2000);
	m_pKinect->GetKinectImage(m_pGraph);

	cv::namedWindow("test", CV_WINDOW_NORMAL);

	cv::imshow("test", *(m_pGraph->ColorImg));

	cv::waitKey(0);

	return true;
}
//����ת�����󾫶�
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
//����UR5����ľ���
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
	TestCalibration();
	

    return 0;
}

