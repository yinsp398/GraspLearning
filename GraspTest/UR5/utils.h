#ifndef UTILS_H
#define UTILS_H

#ifdef UTILSDLL_EXPORTS
#define UTILSDLL_API __declspec(dllexport) 
#else
#define UTILSDLL_API __declspec(dllimport) 
#endif

#include <stdio.h>
#include <string>

/** ***************************************************************************
*  \brief     Represents a pose in 3D
*  \details
*  \author    Axel Rottmann
*  \date      2016-09-2
*  \copyright 2001-2016 by Siemens AG
******************************************************************************/
class UTILSDLL_API Pose3D
{
public:

  /**
  * Constructs the object and initializes all member variables with 0
  */
  Pose3D();

  ~Pose3D();

  /**
  * Wrapper to set the member variables
  */
  void  Set(double X, double Y, double Z, double Rx, double Ry, double Rz);

  /**
  * Calculates the distance between positions (xyz) of two poses
  */
  double  DistEuclidean(const Pose3D& p) const;

  /**
  * Calculates the distance between two poses (taking also the rotation into account)
  */
  double  Dist(const Pose3D& p) const;

  std::string ToString();

  double x;
  double y;
  double z;
  double Rx;
  double Ry;
  double Rz;

};

/** ***************************************************************************
*  \brief     Represents the joints of 6DOF robot arm
*  \details
*  \author    Axel Rottmann
*  \date      2016-09-2
*  \copyright 2001-2016 by Siemens AG
******************************************************************************/
class UTILSDLL_API Joints6DOF
{
public:

  /**
  * Constructs the object and initializes all member variables with 0
  */
  Joints6DOF();

  ~Joints6DOF();

  double Dist(const Joints6DOF& J) const;

  /**
  * Wrapper to set the member variables
  */
  void Set(double Base, double Shoulder, double Elbow, double Wrist1, double Wrist2, double Wrist3);

  std::string ToString();

  double m_Base;
  double m_Shoulder;
  double m_Elbow;
  double m_Wrist1;
  double m_Wrist2;
  double m_Wrist3;

};
#endif  // UTILS_H

