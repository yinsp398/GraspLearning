#pragma once
#include "Common.h"

class Pose3D;
typedef unsigned short UINT16;

class CoordinateMap
{
private:
	ICoordinateMapper *		m_pKinectCoordinate;
	UINT16			  *		m_pDepthBuffer;
public:

	CoordinateMap(ICoordinateMapper *KinectCoordinate);
	~CoordinateMap();

	//���������ϵ�µ�����ת��������������ϵ�����꣬�Ӷ�ȷ��Ŀ����λ��
	GT_RES	Robot2Camera(Pose3D *posCamera, Pose3D *posUR);

	//���������ϵ��ȡ�������ͼ���е�λ��
	GT_RES	Camera2Depth();

	//���������ϵ��λ�û�ȡ���ڲ�ɫͼ���е�λ��
	GT_RES	Camera2Color();

	//�Ӳ�ɫͼ���е�λ�û�ȡ�������ͼ���е�λ��
	GT_RES	Color2Depth(const unsigned int Xc, const unsigned int Yc, unsigned int &Xd, unsigned int &Yd);

	//�Ӳ�ɫͼ���е�λ�õõ�������ʵ�ʵ�ץȡλ�ú���̬(x,y,theta)->(x,y,z,Rx,Ry,Rz)
	GT_RES	ColorDepth2Robot(const GraspPose posColor, Pose3D &posUR);
};