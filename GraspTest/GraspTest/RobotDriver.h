#pragma once
#include "Common.h"

class UR5SocketCom;
class Pose3D;

class RobotDriver
{
private:
	UR5SocketCom *ur5;

	//�ƶ���Ŀ��㣨����ʽ��
	GT_RES	MovetoPose(Pose3D *pos);

	//ִ��ץȡ
	GT_RES	GripperClose();

	//�ж��Ƿ�ץ����
	GT_RES	IsGrasp(bool *IsSuccess);

	//��еצ��
	GT_RES	GripperOpen();
public:
	RobotDriver();
	~RobotDriver();

	//����UR������
	GT_RES	OpenUR();

	//�ر�UR������
	GT_RES	CloseUR();

	//�ƶ���Ŀ�겢ץȡ
	GT_RES	MoveGrasp(Pose3D *pos, bool *success);

};