#ifndef SOCKETSERVEROUT_H
#define SOCKETSERVEROUT_H

#include "socketServer.h"
#include "utils.h"
#include "UR5SocketComCodes.h"

#include <stdio.h>
#include <string>
#include <mutex>
#include <thread> 

enum UR5SocketComOut_SendKey
{
  UR5SocketComOut_SC_DisableRobot       = 0,   // disables any robot motion
  UR5SocketComOut_SC_EnableRobot        = 1,   // enables robot motion
  UR5SocketComOut_SC_PassiveMode        = 2,   // set the communication to passive mode, no robot motion possible
  UR5SocketComOut_SC_ActiveMode         = 3,   // set the communication to active mode
  UR5SocketComOut_SC_ResetTime          = 4,   // resets the UR5 time
  UR5SocketComOut_SC_MoveTCP            = 5,   // moves the TCP to a desired pose
  UR5SocketComOut_SC_MoveJoint          = 6,   // moves the joints to a desired position
  UR5SocketComOut_SC_SetTCP             = 7,   // sets the transformation from the output flangs coordinate system to the TCP as a pose
  UR5SocketComOut_SC_SetPayload         = 8,   // sets the mass and center of grafity of the payload
  UR5SocketComOut_SC_SafetyLimitsPose   = 9,   // is this pose within the safety limits
  UR5SocketComOut_SC_SafetyLimitsJoints = 10,  // is this joint within the safety limits
  //edit by yinsp
  UR5SocketComOut_SC_GripperOpen		= 14,  // to open the gripper
  UR5SocketComOut_SC_GripperClose		= 15,  // to close the gripper
  UR5SocketComOut_SC_GripperCheck		= 16,  // to check if a object is detected in the gripper
  UR5SocketComOut_SC_GripperComplete	= 17,  // to check if the motion of gripper is complete
  UR5SocketComOut_SC_GripperSpeed		= 18   // to set the speed of gripper
  //edit by yinsp
};

//enum UR5SocketComOut_SendKey
//{
//  UR5SocketComOut_SC_SetTCP             = 0,   // sets the transformation from the output flangs coordinate system to the TCP as a pose
//  UR5SocketComOut_SC_SetPayload         = 1,   // sets the mass and center of grafity of the payload
//  UR5SocketComOut_SC_SafetyLimitsPose   = 2,   // is this pose within the safety limits
//  UR5SocketComOut_SC_SafetyLimitsJoints = 10,  // is this joint within the safety limits
//  UR5SocketComOut_SC_MoveTCP            = 3,   // moves the TCP to a desired pose
//  UR5SocketComOut_SC_MoveJoint          = 4,   // moves the joints to a desired position
//  UR5SocketComOut_SC_DisableRobot       = 5,   // disables any robot motion
//  UR5SocketComOut_SC_EnableRobot        = 6,   // enables robot motion
//  UR5SocketComOut_SC_PassiveMode        = 7,   // set the communication to passive mode, no robot motion possible
//  UR5SocketComOut_SC_ActiveMode         = 8,   // set the communication to active mode
//  UR5SocketComOut_SC_ResetTime          = 9    // resets the UR5 time
//};

enum UR5SocketComOut_ReceiveKey
{
  UR5SocketComOut_RC_Accept  = 0,   // the previous command has been accepted
  UR5SocketComOut_RC_Reject  = 1,   // the previous command has been rejected
};

/** ***************************************************************************
*  \brief     Creates socket communication to the UR5 for handling incomming data
*  \details
*  \author    Axel Rottmann
*  \date      2016-08-24
*  \copyright 2001-2016 by Siemens AG
******************************************************************************/
class SocketServerOut : public SocketServer
{
public:

  /**
  * Creates a socket, use \ref Open() to open the server.
  */
  SocketServerOut();

  /**
  * Closes and deletes the socket server.
  */
  ~SocketServerOut();

  bool Init(const char* IPAddress, const char* Port);
  //edit by yinsp
  bool GripperOpen();
  bool GripperClose();
  bool GripperCheck(bool &Check);
  bool GripperComplete(bool &Complete);
  bool GripperSetSpeed(const unsigned int Speed);
  //edit by yinsp
  bool MoveTCP(const Pose3D& P, double BlendRadius, double Time, double Acc, double Speed);
  bool MoveJoint(const Joints6DOF& J, double BlendRadius, double Time, double Acc, double Speed);

  bool SetTCPTransformation(const Pose3D& P);
  bool IsWithinSafetyLimits(const Pose3D& P);
  bool IsWithinSafetyLimits(const Joints6DOF& J);

  bool SetPayload(double Kg, double CoGx, double CoGy, double CoGz);

  bool DisableRobot();
  bool EnableRobot();

  bool PassiveMode();
  bool ActiveMode();

  bool ResetTime();

protected:
  bool SendPoseWithAck(UR5SocketComOut_SendKey Code, const Pose3D& P);
  bool SendWithReply(std::string& SendStr, std::string* ReplyStr);

  /**
  * Sends a command to the robot of a certain length
  *
  * \param SendKey command send key
  * \param ReplyStr if != NULL, awaits a reply from the UR5
  */  
  bool SendCommand1(UR5SocketComOut_SendKey SendKey, std::string* ReplyStr=NULL);
  bool SendCommand7(UR5SocketComOut_SendKey SendKey, 
                     double A, double B, double C, double D, double E, double F, 
                     std::string* ReplyStr=NULL);
  bool SendCommand11(UR5SocketComOut_SendKey SendKey,
                     double A, double B, double C, double D, double E, double F,
                     double Acc, double Speed, double Time, double BlendRadius,
                     std::string* ReplyStr=NULL);

private:

};

#endif  // SOCKETSERVEROUT_H

