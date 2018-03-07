#ifndef UR5SOCKETCOMCODES_H
#define UR5SOCKETCOMCODES_H

//#define USE_CONSOLE

enum UR5SocketCom_Status
{
  UR5SocketCom_NoConnection     = 0,   /*< not connected yet */
  UR5SocketCom_Initializing     = 1,   /*< init phase, waiting for a clients to connect */
  UR5SocketCom_Success          = 2,   /*< connected */
  UR5SocketCom_RobotActive      = 3,   /*< robot is moving */
  UR5SocketCom_RobotIdle        = 4,   /*< robot is idle, await new move command */
  UR5SocketCom_Fault            = 5,   /*< communication fault, connection will be closed */
  UR5SocketCom_ConnectionClosed = 6    /*< connection is closed */
};

#endif  // UR5SOCKETCOMCODES_H

