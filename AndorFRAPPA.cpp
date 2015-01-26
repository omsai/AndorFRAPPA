///////////////////////////////////////////////////////////////////////////////
// FILE:          AndorFRAPPA.cpp
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

// Header for TILL Photonics Laser Scanning Microscopy (LSM) device.
#include "tilllsm.h"

#include "AndorFRAPPA.h"
#include "DeviceUtils.h"
#include "ModuleInterface.h"

// We are defining our own g_Keyword_Port here, because the tillmic.dll library
// opens the serial port and performs communication, and MM must not be allowed
// to communicate on the serial port.  If we instead used the traditional
// MM::g_Keyword_Port, MM would invoke the Serial API and try to open the port.
const char* g_Keyword_Port = "SerialPort";
const char* g_Keyword_GalvoImagingAngle = "GalvoImagingAngle_degrees";
const char* g_Keyword_LaserOnTime = "LaserOnTime_ms";
const char* g_Keyword_ExecutionTime = "ExecutionTime_ms";

const char* g_DeviceName = "AndorFRAPPA";
const char* g_DeviceDescription = "Andor FRAPPA galvo photo-targeting device.";
const char* g_SerialPortDefault = "Undefined";
const char* g_GalvoImagingAngleDefault = "6.5"; // Degrees.
const char* g_TimeDefault = "-999";

/**
 * The galvos have an range of -2^35 to +2^35.  The corresponding physical
 * angle movement is -15.000 to +15.000 degrees.
 */
#define GALVO_RANGE_HALF 34359738368
#define GALVO_RANGE 68719476737
#define GALVO_MIN_DEGREES -15
#define GALVO_MAX_DEGREES 15

#define DWELL_TIME_MIN_US 20
#define DWELL_TIME_MAX_US 1000000

#define TRIGGER_T_OUT 1001
#define TRIGGER_D_OUT 1002
#define TRIGGER_EDGE_RISING 1
#define TRIGGER_EDGE_FALLING -1
#define TRIGGER_EDGE_NONE 0

/**
 * Some of the FRAPPA error code definitions in tilllsm.h up to
 * `LSM_ErrorWrongHandle` use low numbers and, therefore, conflict with
 * Micro-Manager's error messages defined in MMDeviceConstants.h.  The
 * workaround used here is adding an offset to all the FRAPPA error codes.
 */
#define FRAPPA_ERR_CODE_OFFSET 1000

///////////////////////////////////////////////////////////////////////////////
// Exported Module Interface API.

MODULE_API void InitializeModuleData()
{
   RegisterDevice(g_DeviceName, MM::GalvoDevice, g_DeviceDescription);
}

MODULE_API MM::Device* CreateDevice(const char* deviceName)
{
   if (deviceName == 0)
      return 0;

   if (strcmp(deviceName, g_DeviceName) == 0)
   {
      AndorFRAPPA* f = new AndorFRAPPA();
      return f;
   }

   return 0;
}

MODULE_API void DeleteDevice(MM::Device* pDevice)
{
   delete pDevice;
}

///////////////////////////////////////////////////////////////////////////////
// Andor FRAPPA constructor and destructor.

AndorFRAPPA::AndorFRAPPA(void) :
   handle_(0),
   initialized_(false),
   port_(""),
   busy_(false),
   protocolHandle_(0),
   galvoImagingAngle_(atof(g_GalvoImagingAngleDefault)),
   dwellTime_us_(DWELL_TIME_MIN_US),
   regionRepeats_(1),
   laserOnTime_ms_(DWELL_TIME_MIN_US / 1000.0),
   executionTime_ms_(DWELL_TIME_MIN_US / 1000.0)
{
   // Standard error messages.
   InitializeDefaultErrorMessages();

   // Error messges from tilllsm.h.
   SetErrorText(LSM_ErrorUnknown + FRAPPA_ERR_CODE_OFFSET,
                "Generic error code from FRAPPA SDK.");
   SetErrorText(LSM_ErrorNotImplemented + FRAPPA_ERR_CODE_OFFSET,
                "FRAPPA SDK method not implemented yet.");
   std::ostringstream err;
   for (int i = 0; i < 8; i++)
   {
      err.str("");
      err << "Invalid argument " << i + 1 << \
         " supplied to FRAPPA SDK function.";
      SetErrorText(LSM_ErrorInvalidArg1 + i + FRAPPA_ERR_CODE_OFFSET,
                   err.str().c_str());
   }
   SetErrorText(LSM_ErrorWrongHandle + FRAPPA_ERR_CODE_OFFSET,
                "Invalid FRAPPA device handle.");
   SetErrorText(LSM_ErrorNotInitialized + FRAPPA_ERR_CODE_OFFSET,
                "FRAPPA device is not initialized.");
   SetErrorText(LSM_ErrorOpenComPortFailed + FRAPPA_ERR_CODE_OFFSET,
                "Failed to open COM port for FRAPPA.");
   SetErrorText(LSM_ErrorInitFailed + FRAPPA_ERR_CODE_OFFSET,
                "Failed to initialize FRAPPA.");
   SetErrorText(LSM_ErrorNoProperty + FRAPPA_ERR_CODE_OFFSET,
                "No such FRAPPA property.");
   SetErrorText(LSM_ErrorNotCalibrated + FRAPPA_ERR_CODE_OFFSET,
                "FRAPPA device is not calibrated.");
   SetErrorText(LSM_ErrorClearingProtocol + FRAPPA_ERR_CODE_OFFSET,
                "Failed to clear the FRAPPA scan sequence.");
   SetErrorText(LSM_ErrorSetProtocol + FRAPPA_ERR_CODE_OFFSET,
                "Failed to set the new FRAPPA scan sequence.");
   SetErrorText(LSM_ErrorExecProtocol + FRAPPA_ERR_CODE_OFFSET,
                "Failed to start the FRAPPA scan sequence.");
   SetErrorText(LSM_ErrorAbortProtocol + FRAPPA_ERR_CODE_OFFSET,
                "Failed to stop the FRAPPA scan sequence.");
   SetErrorText(LSM_ErrorCurveCalcFailed + FRAPPA_ERR_CODE_OFFSET,
                "FRAPPA failed to determine the blanking time between "
                "scan lines.");
   SetErrorText(LSM_ErrorProtNotSupported + FRAPPA_ERR_CODE_OFFSET,
                "FRAPPA does not support the requested scan sequence.");
   SetErrorText(LSM_ErrorProtCalculation + FRAPPA_ERR_CODE_OFFSET,
                "FRAPPA error creating the requested scan sequence.");
   SetErrorText(LSM_ErrorProtGeometry + FRAPPA_ERR_CODE_OFFSET,
                "FRAPPA error calculating the intersections between "
                "scan lines and polygon to scan.");
   SetErrorText(LSM_ErrorWrongCommandCount + FRAPPA_ERR_CODE_OFFSET,
                "The number of scan commands sent to FRAPPA does not "
                "match the expected number.");

   // Begin pre-initialization properties:
   // 
   // Device Name.
   CreateProperty(MM::g_Keyword_Name, g_DeviceName,
                  MM::String, true);
   // Description.
   CreateProperty(MM::g_Keyword_Description, g_DeviceDescription,
                  MM::String, true);
   // Serial Communication Port.
   CPropertyAction* pAct;
   pAct = new CPropertyAction(this, &AndorFRAPPA::OnPort);
   // FIXME this should be a dropdown list of available serial ports instead of
   // having to enter "COMx" manually.
   CreateProperty(g_Keyword_Port, g_SerialPortDefault,
                  MM::String, false, pAct, true);
   // Galvo Imaging Angle.
   pAct = new CPropertyAction(this, &AndorFRAPPA::OnGalvoImagingAngle);
   CreateProperty(g_Keyword_GalvoImagingAngle, g_GalvoImagingAngleDefault,
                  MM::Float, false, pAct, true);
   SetPropertyLimits(g_Keyword_GalvoImagingAngle, 0, GALVO_MAX_DEGREES);
   // End pre-initialization properties.
}

AndorFRAPPA::~AndorFRAPPA(void)
{
   if (initialized_)
   {
      Shutdown();
   }
}

///////////////////////////////////////////////////////////////////////////////
// Device API functions.

int AndorFRAPPA::Initialize()
{
   int ret;

   ret = LSM_Open(port_.c_str(), &handle_);
   if (ret != LSM_ErrorNone)
   {
      return ret + FRAPPA_ERR_CODE_OFFSET;
   }

   // TODO we could automagically use the existing GetRestPosition and set the
   // default galvo imaging angle to make it easier for users who are migrating
   // from other softwares, or who are adding/removing the FRAPPA to test
   // hardware configurations.

// LSM_AddEndCallback(handle_, (ProtocolDoneCb*) LSMCallBack, NULL);

   // Move galvos to bypass position.
   ret = SetGalvoImagingAngle();
   if (ret != DEVICE_OK)
   {
      return ret;
   }

   // Always return the galvo to bypass position after firing the laser.
   //
   // FIXME scope the T-OUT pin using iQ to keep behavior consistent.
   ret = LSM_ConfigEndTrigger(handle_,
                              TRIGGER_T_OUT, // Pin to indicate firing done.
                              TRIGGER_EDGE_FALLING,
                              true // Return to rest position.
      );

   initialized_ = true;
   
   // Begin normal properties:
   // 
   // Laser-on Time.
   CPropertyAction* pAct;
   pAct = new CPropertyAction(this, &AndorFRAPPA::OnLaserOnTime);
   CreateProperty(g_Keyword_LaserOnTime, g_TimeDefault,
                  MM::Float, true, pAct, false);
   // Execution Time.
   pAct = new CPropertyAction(this, &AndorFRAPPA::OnExecutionTime);
   CreateProperty(g_Keyword_ExecutionTime, g_TimeDefault,
                  MM::Float, true, pAct, false);
   // End normal properties.

   return DEVICE_OK;
}

int AndorFRAPPA::Shutdown()
{
   if (initialized_)
   {
      int ret = LSM_Close(handle_);
      if (ret != LSM_ErrorNone)
      {
         return ret + FRAPPA_ERR_CODE_OFFSET;
      }
      initialized_ = false;
   }

   return DEVICE_OK;
}

void AndorFRAPPA::GetName(char* Name) const
{
   CDeviceUtils::CopyLimitedString(Name, g_DeviceName);
}

bool AndorFRAPPA::Busy()
{
   // TODO check if busy_ is set anywhere in the code.  May be better just to
   // always return "false" to explicitly show behavior if it never changes
   // anywhere else.
   return busy_;
}

///////////////////////////////////////////////////////////////////////////////
// Galvo API functions.

/**
 * Make the device fire at uncalibrated coordinates.  The calibration is done
 * in the Java layer.
 */
int AndorFRAPPA::PointAndFire(double x, double y, double time_us)
{
   time_us = SetDwellTime(time_us);
   LSM_Coordinate c = {x, y};
   int ret = LSM_ShootPoint(handle_, c, time_us, 1, &time_us);
   if (ret != LSM_ErrorNone)
   {
      return ret + FRAPPA_ERR_CODE_OFFSET;
   }

   // Since LSM_ShootPoint is not part of a protocol, the galvos do not return
   // to the rest position, per the returnToRest setting in
   // LSM_ConfigEndTrigger.  Thus we manually move the galvos to bypass
   // position.
   //
   // Strictly speaking, SetGalvoImagingAngle() does an unnecessary step of
   // setting the rest position, whereas here we just need to moves the galvos
   // to the rest position.  But PointAndFire is probably only ever used for
   // calibration and not in protocols, so it doesn't make sense to complicate
   // the code by optimizing.
   ret = SetGalvoImagingAngle();
   if (ret != DEVICE_OK)
   {
      return ret;
   }

   return DEVICE_OK;
}

/**
 * Dwell time is how long the laser should shine on a each voxel of the
 * uploaded region(s).  The FRAPPA dwell time, strictly speaking, is quite
 * different from an interval, but is is the closest match to the Galvo API.
 */
int AndorFRAPPA::SetSpotInterval(double pulseTime_us)
{
   // FIXME petition upstream to change the word interval to be more
   // representative. It's more analogous to "exposure" in the SLM API than an
   // interval.
   SetDwellTime(pulseTime_us);
   return DEVICE_OK;
}

/**
 * SetPosition() of the Galvo API isn't generally useful for the FRAPPA because
 * it will always revert to its "Imaging" reset position.  However, it could
 * help find the uncalibrated center offset spot of the galvos for focusing the
 * laser with the fiber micrometer.
 */
int AndorFRAPPA::SetPosition(double x, double y)
{
   LSM_Coordinate c = {x, y};
   int ret = LSM_SetGalvoRawPosition(handle_, c);
   if (ret != LSM_ErrorNone)
   {
      return ret + FRAPPA_ERR_CODE_OFFSET;
   }
   return DEVICE_OK;
}

/**
 * GetPosition() of the Galvo API doesn't make a whole lot of sense for the
 * FRAPPA because it will always revert to its "Imaging" reset position.  Also,
 * it's not possible to read the current position; only the rest position.
 */
int AndorFRAPPA::GetPosition(double &x, double &y)
{
   LSM_Coordinate c = {x, y};
   int ret = LSM_GetRestPosition(handle_, &c);
   if (ret != LSM_ErrorNone)
   {
      return ret + FRAPPA_ERR_CODE_OFFSET;
   }
   x = c.X;
   y = c.Y;
   return DEVICE_OK;
}

/**
 * The Galvo API template functions are all completely undocumented and thus
 * not clear on what this function is meant to do.
 */
int AndorFRAPPA::SetIlluminationState(bool on)
{
   // FIXME implement this function once upstream explains its purpose.
   return DEVICE_OK;
}

/**
 * The galvos have an raw range of -2^35 to +2^35.  The corresponding physical
 * angle movement is -15.000 to +15.000 degrees.
 * 
 * Returns the raw range which has integer precision.
 */
double AndorFRAPPA::GetXRange()
{
   return (double) GALVO_RANGE;
}

/**
 * The galvos have an raw range of -2^35 to +2^35.  The corresponding physical
 * angle movement is -15.000 to +15.000 degrees.
 * 
 * Returns the raw range which has integer precision.
 */
double AndorFRAPPA::GetYRange()
{
   return (double) GALVO_RANGE;
}

/**
 * Appends polygon vertex at `polygonIndex` to AndorFRAPPA object storage, but
 * does not upload to device.
 */
int AndorFRAPPA::AddPolygonVertex(int polygonIndex, double x, double y)
{
   if (polygons_.size() < (unsigned) (polygonIndex + 1)) {
      polygons_.resize(polygonIndex + 1);
   }
   LSM_Coordinate c = {x, y};
   polygons_.at(polygonIndex).push_back(c);
   return DEVICE_OK;
}

/**
 * Upload 2D polygon(s) to the FRAPPA device.
 *
 * We need to tell the device how much laser time to spend lasing the polygon
 * and how many lines to raster.  This requires knowledge of the image
 * calibration, but for now we use the raw pixels until we figure out how to
 * get the image calibration from the Micro-Manager API.
 */
int AndorFRAPPA::LoadPolygons()
{
   int ret;
   double timeNet_sec, timeGross_sec;
   laserOnTime_ms_ = 0;
   executionTime_ms_ = 0;

   // Build the "protocol" object by adding the polygons.
   for (unsigned int polygonIndex = 0;
        polygonIndex < polygons_.size(); polygonIndex++)
   {
      // timeNet_sec is both an input and an output value from
      // LSM_AddPolygon(), so the return value can be different.
      timeNet_sec = dwellTime_us_ / 1000000.0;
      timeGross_sec = 0.0;      // Might not be implemented in FRAPPA API per
                                // test program included with FRAPPA SDK.

      // Use the appropriate LSMM_Add* function for point, line and polygon
      // regions, respectively.
      switch(polygons_.at(polygonIndex).size())
      {
         // TODO compare behavior of regionRepeats_ in LSM_Add*() versus
         // LSM_Load().  It might be better to allow the user to choose to
         // repeat regions first, repeat after all regions, or even separate
         // repeats for both.
         case 1:
            // TODO loop cycle is undocumented. I assume it is the cycle time
            // between how often regionRepeats_ is looped.
            ret = LSM_AddPoint(handle_, &protocolHandle_,
                               polygons_.at(polygonIndex)[0],
                               &timeNet_sec,   // Laser on time.
                               &timeGross_sec, // Execution time.
                               regionRepeats_, // Loops.
                               0.0             // Loop cycle.
               );
            break;
         case 2:
            // Bi-directional mode has been set off for the effect on the
            // biological specimen to be position independent, which is more
            // important than any speed gain.
            ret = LSM_AddLine(handle_, &protocolHandle_,
                              polygons_.at(polygonIndex)[0], // Coordinate 1.
                              polygons_.at(polygonIndex)[1], // Coordinate 2.
                              false,                         // Bi-directional.
                              &timeNet_sec,                  // Laser on time.
                              &timeGross_sec,                // Execution time.
                              regionRepeats_                 // Loops.
               );
            break;
         default:
            // TODO we must be able to distinguish between a line polygon and
            // 2D polygon.  Perhaps one could check if the first and last point
            // overlap; more investigation is needed.
            ret = LSM_AddPolygon(handle_, &protocolHandle_,
                                 (LSM_Coordinate*)&(polygons_.at(polygonIndex)),
                                 polygons_.at(polygonIndex).size(),
                                 0,              // Alpha.
                                 false,          // Counter clockwise.
                                 10,             // Lines. FIXME constant!
                                 &timeNet_sec,   // Laser on time.
                                 &timeGross_sec, // Execution time.
                                 regionRepeats_  // Loops.
               );
            if (ret != LSM_ErrorNone)
            {
               return ret + FRAPPA_ERR_CODE_OFFSET;
            }
            break;
      } // End switch.

      laserOnTime_ms_ += timeNet_sec * 1000;
      executionTime_ms_ += timeGross_sec * 1000;
   } // Finished adding all polygons to "protocol".

   // Upload protocol to the device.
   ret = LSM_Load(handle_, protocolHandle_,
                  1             // Loops.
      );
   if (ret != LSM_ErrorNone)
   {
      return ret + FRAPPA_ERR_CODE_OFFSET;
   }
   return DEVICE_OK;
}

/**
 * Delete all polygons from AndorFRAPPA object storage and from device.
 */
int AndorFRAPPA::DeletePolygons()
{
   // Reset times to reflect single point photo-targeting.
   laserOnTime_ms_ = dwellTime_us_ / 1000.0;
   executionTime_ms_ = laserOnTime_ms_;

   polygons_.clear();
   int ret = LSM_DeleteProtocol(handle_, protocolHandle_);
   if (ret != LSM_ErrorNone)
   {
      return ret + FRAPPA_ERR_CODE_OFFSET;
   }
   laserOnTime_ms_ = 0;
   return DEVICE_OK;
}

/**
 * Set repeats for 2D polygon(s) already uploaded to FRAPPA.
 */
int AndorFRAPPA::SetPolygonRepetitions(int repetitions)
{
   regionRepeats_ = repetitions;
   return DEVICE_OK;
}

/**
 * Execute 2D polygon(s) already uploaded to FRAPPA.
 */
int AndorFRAPPA::RunPolygons()
{
   // FIXME the Galvo API does not state what the distinction between this and
   // RunSequence().  Clarify with upstream.
   int ret = LSM_Execute(handle_);
   if (ret != LSM_ErrorNone)
   {
      return ret + FRAPPA_ERR_CODE_OFFSET;
   }

   return DEVICE_OK;
}

/**
 * Execute 2D polygon(s) "protocol" already uploaded to FRAPPA.
 */
int AndorFRAPPA::RunSequence()
{
   // FIXME upstream needs to clarify the purpose of this function in the
   // GalvoAPI.  "Sequence" may refer to hardware synchronization, but perhaps
   // there is a distinction between sequecne and sequencing.
   int ret = LSM_Execute(handle_);
   if (ret != LSM_ErrorNone)
   {
      return ret + FRAPPA_ERR_CODE_OFFSET;
   }

   return DEVICE_OK;
}

/**
 * Stop a running FRAPPA "protocol" execution.
 */
int AndorFRAPPA::StopSequence()
{
   int ret = LSM_Abort(handle_);
   if (ret != LSM_ErrorNone)
   {
      return ret + FRAPPA_ERR_CODE_OFFSET;
   }

   return DEVICE_OK;
}

/**
 * ???
 */
int AndorFRAPPA::GetChannel(char* channelName)
{
   // FIXME the Galvo API does not state the purpose of this function.
   return DEVICE_OK;
}

///////////////////////////////////////////////////////////////////////////////
// Property Action Handlers. 

/**
 * Sets FRAPPA serial port during pre-initialization.
 */
int AndorFRAPPA::OnPort(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::BeforeGet)
   {
      pProp->Set(port_.c_str());
   }
   else if (eAct == MM::AfterSet)
   {
      if (initialized_)
      {
         // Revert.
         pProp->Set(port_.c_str());
      }
     else
      {
         pProp->Get(port_);
      }
   }

   return DEVICE_OK;
}

/**
 * GalvoImagingAngle needs to be set in cases where the device is physically
 * placed along the camera imaging light path.  The X-galvo does double duty:
 * it does photo-targeting and also acts as a mirror along the 4F light path.
 * The Y-galvo is also moved to an extreme to prevent any laser leakage from
 * reaching the specimen. The X-galvo bypass angle is typically between +6.000
 * and +7.000 degrees.
 */
int AndorFRAPPA::OnGalvoImagingAngle(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::BeforeGet)
   {
      pProp->Set(galvoImagingAngle_);
   }
   else if (eAct == MM::AfterSet)
   {
      pProp->Get(galvoImagingAngle_);
      if (initialized_)
      {
         int ret;
         ret = SetGalvoImagingAngle();
         if (ret != DEVICE_OK)
         {
            return ret + FRAPPA_ERR_CODE_OFFSET;
         }
      }
      pProp->Set(galvoImagingAngle_);
   }

   return DEVICE_OK;
}

/**
 * Total time laser will be shining on uploaded region(s).
 *
 * Read-only property.
 */
int AndorFRAPPA::OnLaserOnTime(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::BeforeGet)
   {
      pProp->Set(laserOnTime_ms_ * (double) regionRepeats_);
   }

   return DEVICE_OK;
}

/**
 * Total execution time for uploaded region(s).  This will state a lower value
 * than actual, since the FRAPPA API does not calculate the galvo movement
 * delays between regions.
 *
 * Read-only property.
 */
int AndorFRAPPA::OnExecutionTime(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::BeforeGet)
   {
      pProp->Set(executionTime_ms_ * (double) regionRepeats_);
   }

   return DEVICE_OK;
}

///////////////////////////////////////////////////////////////////////////////
// Helper functions.

/**
 * Converts galvos angle range of -15.000 to +15.000 degrees to it's native raw
 * arbitrary units of -2^35 to +2^35.
 */
double AndorFRAPPA::GalvoDegreesToRawPosition(double degrees)
{
   return degrees * GALVO_RANGE_HALF / GALVO_MAX_DEGREES;
}

/**
 * Converts galvos raw arbitrary units of -2^35 to +2^35 to physical angle
 * range of -15.000 to +15.000 degrees.
 */
double AndorFRAPPA::GalvoRawPositionToDegrees(double position)
{
   return position / GALVO_RANGE_HALF * GALVO_MAX_DEGREES;
}

/**
 * Move galvos to preset imaging angle for bypass mode.
 */
int AndorFRAPPA::SetGalvoImagingAngle()
{
   int ret;
   LSM_Coordinate c;

   // Move to the pre-initialization angle provided by the user.
   c.X = GalvoDegreesToRawPosition(galvoImagingAngle_);
   c.Y = GalvoDegreesToRawPosition(GALVO_MIN_DEGREES);
   ret = LSM_SetRestPosition(handle_, c);
   if (ret != LSM_ErrorNone)
   {
      return ret + FRAPPA_ERR_CODE_OFFSET;
   }

   // Validate the actual angle position.
   ret = LSM_GetRestPosition(handle_, &c);
   if (ret != LSM_ErrorNone)
   {
      return ret + FRAPPA_ERR_CODE_OFFSET;
   }
   galvoImagingAngle_ = GalvoRawPositionToDegrees(c.X);

   // Move the galvo now for immediate effect.
   ret = LSM_SetGalvoRawPosition(handle_, c);
   if (ret != LSM_ErrorNone)
   {
      return ret + FRAPPA_ERR_CODE_OFFSET;
   }

   return DEVICE_OK;
}

/**
 * Cooerce dwell time to valid value, otherwise the FRAPPA SDK gives an error.
 */
double AndorFRAPPA::SetDwellTime(double time_us)
{
   // FIXME this type of range checking should be done in the GUI; or at least
   // the max value should be checked in the GUI.
   if (time_us < DWELL_TIME_MIN_US)
   {
      time_us = DWELL_TIME_MIN_US;
   }
   else if (time_us > DWELL_TIME_MAX_US)
   {
      time_us = DWELL_TIME_MAX_US;
   }

   dwellTime_us_ = time_us;
   return time_us;
}
