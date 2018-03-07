#ifndef SOCKETSERVER_H
#define SOCKETSERVER_H

#include <stdio.h>
#include <winsock2.h>
#include <string>

/** ***************************************************************************
*  \brief     Creates and opens a socket server
*  \details
*  \author    Axel Rottmann
*  \date      2016-08-24
*  \copyright 2001-2016 by Siemens AG
******************************************************************************/
class SocketServer
{
public:

  /**
  * Creates a socket, use Open() to open the server.
  */
  SocketServer();

  /**
  * Closes and deletes the socket server.
  */
  ~SocketServer();

  /**
  * Opens a socket server.
  */
  bool Open(const char* IPAddress, const char* Port);

  /**
  * Close the server.
  */
  bool Close();

  /**
  * Start up WSA.
  */
  static bool InitWSA();

  /**
  * Clean up WSA.
  */
  static void CloseWSA();

  /**
  * Sends a string to the socket
  */
  bool Send(const std::string str);

  /**
  * Receives a string from the socket.
  */
  int Receive(std::string& str);

  /**
  * Gets the error of the command that failed
  */
  bool GetLastError(std::string& ErrorMsg);
  std::string GetLastError();

protected:

  void SetErrorMsg(const char *fmt, ...);
  void ClearErrorMsg();

  SOCKET      m_Socket;
  std::string m_ErrorMsg;

};

#endif  // SOCKETSERVER_H

