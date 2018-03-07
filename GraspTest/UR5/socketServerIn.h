#ifndef SOCKETSERVERIN_H
#define SOCKETSERVERIN_H

#include "socketServer.h"
#include "utils.h"
#include "UR5SocketComCodes.h"

#include <stdio.h>
#include <string>
#include <mutex>  

enum UR5SocketComIn_ReceiveKey
{
  UR5SocketComIn_RC_Status        = 0,   // status of the UR5
  UR5SocketComIn_RC_TCPPose       = 1,   // current TCP pose
  UR5SocketComIn_RC_TCPSpeed      = 2,   // current TCP speed
  UR5SocketComIn_RC_JointPosition = 3,   // current joint positions
  UR5SocketComIn_RC_JointSpeed    = 4,   // current joint speeds
  UR5SocketComIn_RC_JointTorque   = 5,   // current joint torques
  UR5SocketComIn_RC_UR5Time       = 6    // time in sec
};

/** ***************************************************************************
*  \brief     Creates socket communication to the UR5 for handling incomming data
*  \details
*  \author    Axel Rottmann
*  \date      2016-08-24
*  \copyright 2001-2016 by Siemens AG
******************************************************************************/
class SocketServerIn : public SocketServer
{
public:

  /**
  * Creates a socket, use Open() to open the server.
  */
  SocketServerIn();

  /**
  * Closes and deletes the socket server.
  */
  ~SocketServerIn();

  bool Init(const char* IPAddress, const char* Port);

  bool OpenFile(const char* Filename);

  static void CycleStatic(SocketServerIn* ssi);

  void SetRobotIdle();

  Pose3D              GetTCPPose();
  Pose3D              GetTCPSpeed();
  Joints6DOF          GetJointPosition();
  Joints6DOF          GetJointSpeed();
  Joints6DOF          GetJointTorque();
  UR5SocketCom_Status GetStatus();
  double              GetUR5Time();

protected:
  void SetTCPPose(const Pose3D& P);
  void SetTCPSpeed(const Pose3D& V);
  void SetJointPosition(const Joints6DOF& J);
  void SetJointSpeed(const Joints6DOF& S);
  void SetJointTorque(const Joints6DOF& T);
  void SetStatus(UR5SocketCom_Status Status);
  void SetUR5Time(double Time);

  bool ParsePose3D(std::stringstream& StrStr, Pose3D& Pose);
  bool ParseJoint6DOF(std::stringstream& StrStr, Joints6DOF& Joint);
  bool Parse6Double(std::stringstream& StrStr, double& A, double& B, double& C, double& D, double& E, double& F);

  bool ParseInput(const std::string str);
  void Cycle();

private:

  double      m_UR5Time;
  std::mutex  m_UR5Time_Mutex;

  Pose3D      m_TCPPose;
  std::mutex  m_TCPPose_Mutex;

  Pose3D      m_TCPSpeed;
  std::mutex  m_TCPSpeed_Mutex;

  Joints6DOF  m_JointPosition;
  std::mutex  m_JointPosition_Mutex;

  Joints6DOF  m_JointSpeed;
  std::mutex  m_JointSpeed_Mutex;

  Joints6DOF  m_JointTorque;
  std::mutex  m_JointTorque_Mutex;

  UR5SocketCom_Status m_Status;
  std::mutex          m_Status_Mutex;

  FILE*               m_pFileUR5Data;
  UR5SocketCom_Status m_PreviousStatus;
};

#endif  // SOCKETSERVERIN_H

