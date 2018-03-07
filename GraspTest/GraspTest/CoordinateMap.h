#pragma once
#include "Common.h"

class Pose3D;

class CoordinateMap
{
private:
	//ICoordinateMapper *		m_pKinectCoordinate;
public:

	CoordinateMap();
	~CoordinateMap();

	//���������ϵ�µ�����ת��������������ϵ�����꣬�Ӷ�ȷ��Ŀ����λ��
	GT_RES	Robot2Camera(Pose3D *posCamera, Pose3D *posUR);

	//���������ϵ��ȡ�������ͼ���е�λ��
	GT_RES	Camera2Depth();

	//���������ϵ��λ�û�ȡ���ڲ�ɫͼ���е�λ��
	GT_RES	Camera2Color();

	//�Ӳ�ɫͼ���е�λ�û�ȡ�������ͼ���е�λ��
	GT_RES	CoordinateMap::Color2Depth(const unsigned int Xc, const unsigned int Yc, unsigned int &Xd, unsigned int &Yd);

	//�Ӳ�ɫͼ���е�λ�õõ�������ʵ�ʵ�ץȡλ�ú���̬(x,y,theta)->(x,y,z,Rx,Ry,Rz)
	GT_RES	ColorDepth2Robot(const GraspPose posColor, const float Depth, Pose3D &posUR);
};