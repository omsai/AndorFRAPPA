///////////////////////////////////////////////////////////////////////////////
// FILE:          AndorFRAPPA.h
// PROJECT:       Micro-Manager
// SUBSYSTEM:     DeviceAdapters
//-----------------------------------------------------------------------------
// DESCRIPTION:   Andor FRAPPA adapter control using RS-232 via
//                tillmic{,64}.dll.
//
// COPYRIGHT:     Pariksheet Nanda, 2014-2015
//
// LICENSE:       LGPL-3+
//
//                This file is part of AndorFRAPPA.
//
//                AndorFRAPPA is free software: you can redistribute it and/or
//                modify it under the terms of the GNU Lesser General Public
//                License as published by the Free Software Foundation, either
//                version 3 of the License, or (at your option) any later
//                version.
//
//                AndorFRAPPA is distributed in the hope that it will be useful,
//                but WITHOUT ANY WARRANTY; without even the implied warranty of
//                MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//                GNU Lesser General Public License for more details.
//
//                You should have received a copy of the GNU Lesser General
//                Public License along with AndorFRAPPA.  If not, see
//                <http://www.gnu.org/licenses/>.
//
// AUTHORS:       Pariksheet Nanda <omsai@member.fsf.org>

#ifndef _AndorFRAPPA_H_
#define _AndorFRAPPA_H_

#include "DeviceBase.h"
#include "MMDevice.h"
#include <string>

typedef std::vector<LSM_Coordinate> LSM_Region;


class AndorFRAPPA : public CGalvoBase<AndorFRAPPA>
{
public:
   AndorFRAPPA(void);
   ~AndorFRAPPA(void);
  
   // Device API.
   int Initialize();
   int Shutdown();
   void GetName(char* pszName) const;
   bool Busy();

  // Galvo API.
   int PointAndFire(double x, double y, double time_us);
   int SetSpotInterval(double pulseInterval_us);
   int SetPosition(double x, double y);
   int GetPosition(double& x, double& y);
   int SetIlluminationState(bool on);
   double GetXRange();
   double GetYRange();
   int AddPolygonVertex(int polygonIndex, double x, double y);
   int DeletePolygons();
   int RunSequence();
   int LoadPolygons();
   int SetPolygonRepetitions(int repetitions);
   int RunPolygons();
   int StopSequence();
   int GetChannel(char* channelName);

  // Property Action Handlers.
   int OnPort(MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnGalvoImagingAngle(MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnLaserOnTime(MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnExecutionTime(MM::PropertyBase* pProp, MM::ActionType eAct);
// int OnCalibrationMode(MM::PropertyBase* pProp, MM::ActionType eAct);

private:
   void* handle_;
   bool initialized_;
   std::string port_;
   bool busy_;
   long calibrationMode_;
   /**
    * A TILL protocol handle is required when any 2D region needs to be
    * uploaded to the FRAPPA.
    */
   void* protocolHandle_;
   std::vector<LSM_Region> polygons_;
   double currentX_;
   double currentY_;
   double galvoImagingAngle_;
   double dwellTime_us_;
   long regionRepeats_;
   double laserOnTime_ms_;
   double executionTime_ms_;


  // Helper functions.
   double GalvoDegreesToRawPosition(double degrees);
   double GalvoRawPositionToDegrees(double position);
   int SetGalvoImagingAngle();
   double SetDwellTime(double time_us);
   //int SafeStoreSequence(tStringList sequenceList);
   //void RunDummyCalibration(bool laser2);
};

#endif	// _AndorFRAPPA_H_
