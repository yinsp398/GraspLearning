#ifndef UR5SOCKETCOM_H
#define UR5SOCKETCOM_H

#ifdef UR5SOCKETCOMDLL_EXPORTS
#define UR5SOCKETCOMDLL_API __declspec(dllexport) 
#else
#define UR5SOCKETCOMDLL_API __declspec(dllimport) 
#endif


#include "UR5SocketComCodes.h"
#include "utils.h"

#include "socketServerIn.h"
#include "socketServerOut.h"

#include <stdio.h>
#include <thread>

/** ***************************************************************************
*  \brief     Interface to communicate with the UR5 using a socket communication
*  \details
*  \author    Axel Rottmann
*  \date      2016-08-24
*  \copyright 2001-2016 by Siemens AG
******************************************************************************/
class UR5SOCKETCOMDLL_API UR5SocketCom
{
public:

  UR5SocketCom();

  ~UR5SocketCom();

  /**
  * Initializes the socket connections.
  */
  bool Init(const char* IPAddress, const char* PortIn, const char* PortOut);

  /**
  * Gets the error of the command that failed
  */
  bool        GetLastError(std::string& ErrorMsg);
  std::string GetLastError();

  /**
  * All data from the UR5 will be printed to a file.
  */
  bool SetOutputFile(const char* Filename = "UR5.dat");

  /**
  * Disables any robot motion. If the robot is in motion, it will be slow down.
  */
  bool DisableRobot();

  /**
  * Enables robot motion. At startup, the robot will be disabled, \see DisableRobot()
  */
  bool EnableRobot();

  /**
  * Resets UR5 time.
  */
  bool ResetTime();

  /**
  * Sets the communication to passive mode. In this mode all move commands will be ignored by the robot. 
  * This mode is useful to move the robot in Polyscope.
  */
  bool PassiveMode();

  /**
  * Sets the communication to active mode. In this mode the robot should only me move using this communication.
  */
  bool ActiveMode();

  /**
  * Checks if the given pose is reachable and within the current safety limits of the robot.
  * This check considers joint limmits, safety plane limits, TCP orientation deviation limits and
  * range of the robot. 
  */
  bool IsWithinSafetyLimits(const Pose3D& P);

  //edit by yinsp
  bool GripperOpen();
  bool GripperClose();
  bool IsObjectdetected(bool &Detected);
  bool IsGripperMotionCompleted(bool &Completed);
  bool SetGripperSpeed(const unsigned int Speed);
  //edit by yinsp

  /**
  * Checks if the given joint position is reachable and within the current safety limits of the robot.
  * This check considers joint limmits, safety plane limits, TCP orientation deviation limits and
  * range of the robot.
  */
  bool IsWithinSafetyLimits(const Joints6DOF& J);

  /**
  * Returns the current measured tool pose.
  * The 6D pose [X Y Z Roll Pitch Yaw] representing the tool position and orientation specified in the base frame.
  * In matrix form the RPY vector is defined as Rrpy = Rz(yaw)Ry(pitch)Rx(roll).
  *
  * \return TCP pose
  */
  Pose3D GetTCPPose();
  void   GetTCPPose(double& X, double& Y, double& Z, double& Rx, double& Ry, double& Rz);

  /**
  * Returns the current measured tool speed.
  * The speed of the TCP is returned in a pose structure. The first three values are the cartesian speeds along x,y,z
  * and the last three define the current rotations roll,pitch,yaw in radians/s. In matrix form the RPY vector is 
  * defined as Rrpy = Rz(yaw)Ry(pitch)Rx(roll).
  *
  * \return TCP speed
  */
  Pose3D GetTCPSpeed();
  void   GetTCPSpeed(double& X, double& Y, double& Z, double& Rx, double& Ry, double& Rz);

  /**
  * Returns the actual angular positions of all joints.
  * The angular actual positions are expressed in radians.
  *
  * \return Joint position in rad
  */
  Joints6DOF GetJointPosition();
  void       GetJointPosition(double& Base, double& Shoulder, double& Elbow, double& Wrist1, double& Wrist2, double& Wrist3);

  /**
  * Returns the actual angular velocities of all joints.
  *
  * \return Joint speed in rad/s
  */
  Joints6DOF GetJointSpeed();
  void       GetJointSpeed(double& Base, double& Shoulder, double& Elbow, double& Wrist1, double& Wrist2, double& Wrist3);

  /**
  * Returns the actual torques of all joints.
  * The torque on the joints, corrected by the torque needed to move the robot itself (gravoty, friction, etc.).
  *
  * \return Joint torque
  */
  Joints6DOF GetJointTorque();
  void       GetJointTorque(double& Base, double& Shoulder, double& Elbow, double& Wrist1, double& Wrist2, double& Wrist3);

  /**
  * Returns the UR5 time since program start
  */
  double GetUR5Time();

  UR5SocketCom_Status    GetStatus();

  static std::string Status2Str(UR5SocketCom_Status S);

  /**
  * Sets the tool center point.
  * Sets the transformation from the output flange coordinate system to the TCP as a pose.
  *
  * \param P TCP pose
  */
  bool SetTCPTransformation(const Pose3D& P);
  bool SetTCPTransformation(double X, double Y, double Z, double Rx, double Ry, double Rz);

  /**
  * Moves the tool center point to a certain pose (linear in tool-space).
  * If you want the robot to stop at that position, set the blend radius to 0. If you want that the robot
  * constantly moves to further positions, you need to set a blend radius.
  *
  * \param P 6D pose (tool position [m] and orientation of RPY [rad] specified in the base frame). In matrix form
  *          the RPY vector is defined as Rrpy = Rz(yaw)Ry(pitch)Rx(roll).
  * \param BlendRadius Blend radius [m]. The robots stops if the paremeter is 0.
  * \param Time Time [s]. Time setting has priority over speed and acceleration setting.
  * \param Acc Tool acceleration [m/s^2]
  * \param Speed Tool speed [m/s]
  */
  bool MoveTCP(const Pose3D& P,
               double BlendRadius=0.0, double Time=0.0, double Acc=1.2, double Speed=0.25);
  bool MoveTCP(double X, double Y, double Z, double Rx, double Ry, double Rz,
               double BlendRadius=0.0, double Time=0.0, double Acc=1.2, double Speed=0.25);

  /**
  * Moves the joints to a certain position (linear in joint-space).
  * If you want the robot to stop at that position, set the blend radius to 0. If you want that the robot
  * constantly moves to further positions, you need to set a blend radius.
  *
  * \param J 6DOF joints [rad]
  * \param BlendRadius Blend radius [m]. The robots stops if the paremeter is 0.
  * \param Time Time [s]. Time setting has priority over speed and acceleration setting.
  * \param Acc Tool acceleration [m/s^2]
  * \param Speed Tool speed [m/s]
  */
  bool MoveJoint(const Joints6DOF& J,
                 double BlendRadius=0.0, double Time=0.0, double Acc=1.4, double Speed=1.05);
  bool MoveJoint(double Base, double Shoulder, double Elbow, double Wrist1, double Wrist2, double Wrist3,
                 double BlendRadius=0.0, double Time=0.0, double Acc=1.4, double Speed=1.05);


  /**
  * Sets the mass and center of grafity of the payload.
  *
  * \param Kg payload in kg
  * \param CoGx center of grafity, displacement from the toolmount in x direction (in meter)
  * \param CoGy center of grafity, displacement from the toolmount in y direction (in meter)
  * \param CoGz center of grafity, displacement from the toolmount in z direction (in meter)
  */
  bool SetPayload(double Kg, double CoGx, double CoGy, double CoGz);


private:

  void ClearErrorMsg();
  bool SocketOutHandling(bool Success);

  //SocketServerIn  m_SocketIn;
  // SocketServerOut m_SocketOut;
  // std::thread     m_ThrdSocketIn;
};

#endif  // UR5SOCKETCOM_H

