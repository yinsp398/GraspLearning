#pragma once
#include "Common.h"

class UR5SocketCom;
class Pose3D;

class RobotDriver
{
private:
	UR5SocketCom *ur5;

	//移动到目标点（阻塞式）
	GT_RES	MovetoPose(Pose3D *pos);

	//执行抓取
	GT_RES	GripperClose();

	//判断是否抓到了
	GT_RES	IsGrasp(bool *IsSuccess);

	//机械爪打开
	GT_RES	GripperOpen();
public:
	RobotDriver();
	~RobotDriver();

	//启动UR机器人
	GT_RES	OpenUR();

	//关闭UR机器人
	GT_RES	CloseUR();

	//移动到目标并抓取
	GT_RES	MoveGrasp(Pose3D *pos, bool *success);

};