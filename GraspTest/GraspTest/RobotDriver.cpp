

#include "RobotDriver.h"

#include "UR5SocketCom.h"
#include "UR5SocketComCodes.h"
#include "utils.h"
#include <iostream>

RobotDriver::RobotDriver()
{
	ur5 = new UR5SocketCom;
}

RobotDriver::~RobotDriver()
{
	delete ur5;
	ur5 = NULL;
}
//����UR������
GT_RES RobotDriver::OpenUR()
{
	//��UR5������ͨ�Ŵ�
	if (!(ur5->Init(IPADDR, PORT1, PORT2)))
	{
		printf("failed to open UR5 connection: '%s'\n",ur5->GetLastError().c_str());
		return GT_RES_NODEVICE;
	}
	printf("ur5 init OK!\n");
	//�ȴ�ͨ�Ž����ɹ�
	while ((ur5->GetStatus()) != UR5SocketCom_RobotIdle)
	{
		Sleep(8);
	}
	//����UR5�˶���¼����ĵ�
	ur5->SetOutputFile();
	//����UR5������
	if (!(ur5->EnableRobot()))
	{
		printf("enable UR5 failed: '%s'\n",ur5->GetLastError().c_str());
		return GT_RES_DEVICEERROR;
	}
	//����TCP
	Pose3D	define_TCP;
	define_TCP.Set(TCPPOSE);
	if (!(ur5->SetTCPTransformation(define_TCP)))
	{
		printf("define TCP failed: '%s'\n", ur5->GetLastError().c_str());
		return GT_RES_DEVICEERROR;
	}
	//���ø�������
	if (!(ur5->SetPayload(PAYLOAD)))
	{
		printf("define payload failed: '%s'\n",ur5->GetLastError().c_str());
		return GT_RES_DEVICEERROR;
	}
	return GT_RES_OK;
}


//�ر�UR������
GT_RES	RobotDriver::CloseUR()
{
	if (!ur5->DisableRobot())
	{
		printf("disable UR5 failed: '%s'\n",ur5->GetLastError().c_str());
		return GT_RES_DEVICEERROR;
	}
	return GT_RES_OK;

}

//�ƶ���Ŀ��㲢ץȡ
GT_RES	RobotDriver::MoveGrasp(Pose3D *pos, bool *success)
{
	Pose3D pos1,pos2;
	GT_RES res;
	pos1.Set(POSE1);
	pos2.Set(POSE2);
	res = MovetoPose(&pos2);
	if (res != GT_RES_OK)
	{
		return res;
	}
	res = MovetoPose(pos);
	if (res != GT_RES_OK)
	{
		return res;
	}
	res = GripperClose();
	if (res != GT_RES_OK)
	{
		return res;
	}
	res = MovetoPose(&pos2);
	if (res != GT_RES_OK)
	{
		return res;
	}
	res = GripperClose();
	if (res != GT_RES_OK)
	{
		return res;
	}
	res = IsGrasp(success);
	if (res != GT_RES_OK)
	{
		return res;
	}
	res = GripperOpen();
	if (res != GT_RES_OK)
	{
		return res;
	}
	res = MovetoPose(&pos1);
	if (res != GT_RES_OK)
	{
		return res;
	}
	return GT_RES_OK;
}

//�ƶ���Ŀ��㣨����ʽ��
GT_RES	RobotDriver::MovetoPose(Pose3D *pos)
{
	UR5SocketCom_Status status = ur5->GetStatus();
	bool res = false;
	if (status == UR5SocketCom_Fault || status == UR5SocketCom_ConnectionClosed)
	{
		printf("UR5 Robot's status is %u.\n", status);
		return GT_RES_DEVICEERROR;
	}
	else if (status != UR5SocketCom_RobotIdle && status != UR5SocketCom_Success)
	{
		printf("UR5 Robot's status is %u.\n", status);
		return GT_RES_DEVICENOTREADY;
	}
	res = ur5->MoveTCP(*pos);
	if (!res)
	{
		printf("MoveTCP failed!\n");
		return GT_RES_DEVICEERROR;
	}
	Sleep(50);	//Wait until Robot change to Move station
	while ((status = ur5->GetStatus()) != UR5SocketCom_RobotIdle)
	{
		Sleep(8);
	}
	return GT_RES_OK;
}

//ִ��ץȡ
GT_RES	RobotDriver::GripperClose()
{
	bool Completed = false;
	if (!ur5->GripperClose())
		return GT_RES_ERROR;
	while (!Completed)
	{
		if (!ur5->IsGripperMotionCompleted(Completed))
			return GT_RES_ERROR;
	}
	return GT_RES_OK;
}

//�ж��Ƿ�ץ����
GT_RES	RobotDriver::IsGrasp(bool *IsSuccess)
{
	if (ur5->IsObjectdetected(*IsSuccess))
		return GT_RES_OK;
	else
		return GT_RES_ERROR;
}

GT_RES	RobotDriver::GripperOpen()
{
	bool Completed = false;
	if (!ur5->GripperOpen())
		return GT_RES_ERROR;
	while (!Completed)
	{
		if (!ur5->IsGripperMotionCompleted(Completed))
			return GT_RES_ERROR;
	}
	return GT_RES_OK;
}