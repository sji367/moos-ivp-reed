// $Header: /raid/cvs-server/REPOSITORY/software/MOOS/interface/general/iGPS/GPSInstrument.cpp,v 5.18 2007/01/31 03:53:10 anrp Exp $
// copyright (2001-2003) Massachusetts Institute of Technology (pnewman et al.)

// GPSInstrument.cpp: implementation of the CGPS_MB1 class.
// 
// Receives data from the GPS in the MB1 RTA, which are passed via UDP
// packets with NMEA strings starting at byte 31
//////////////////////////////////////////////////////////////////////
#include <cstring>
#include "OS5000.h"
#include "MBUtils.h"

using namespace std;

#define DEBUG true

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

iOS5000::iOS5000()
{
  m_sType = "VANILLA";
  m_bIgnoreNumSats = false;
}

iOS5000::~iOS5000()
{
}


/////////////////////////////////////////////
///this is where it all happens..
bool iOS5000::Iterate()
{
  if (GetData()) {
    PublishData();
  }

  return true;
}


bool iOS5000::OnStartUp()
{
  CMOOSInstrument::OnStartUp();

  //set up Geodetic conversions
  double dfLatOrigin;
  double dfLongOrigin;


  m_MissionReader.GetConfigurationParam("TYPE", m_sType);

  string sVal;

  if (m_MissionReader.GetValue("LatOrigin", sVal))
    {
      dfLatOrigin = atof(sVal.c_str());
    }
  else
    {
      MOOSTrace("LatOrigin not set - FAIL\n");
      return false;
    }

  if (m_MissionReader.GetValue("LongOrigin", sVal))
    {
      dfLongOrigin = atof(sVal.c_str());
    }
  else
    {
      MOOSTrace("LongOrigin not set - FAIL\n");
      return false;
    }

  if (!m_Geodesy.Initialise(dfLatOrigin, dfLongOrigin))
    {
      MOOSTrace("Geodesy Init failed - FAIL\n");
      return false;
    }

  //here we make the variables that we are managing
  double dfOS5000Period = 0.01;

  //GPS update @ 10Hz
  AddMOOSVariable("C", "", "OS5000_HEADING", dfOS5000Period);

  AddMOOSVariable("P", "", "Pitch", dfOS5000Period);

  AddMOOSVariable("R", "", "Roll", dfOS5000Period);

  AddMOOSVariable("T", "", "Temp", dfOS5000Period);

  AddMOOSVariable("Raw", "", "OS5000_RAW", dfOS5000Period);


  if (IsSimulateMode())
    {
      //not much to do...
      RegisterMOOSVariables();
    }
  else
    {
      if (!SetupPort()) 
	return false;
    
      //try 10 times to initialise sensor
      if (!InitialiseSensorN(10, "OS5000")) 
	return false;
    
    }
  return true;
}

bool iOS5000::OnNewMail(MOOSMSG_LIST &NewMail)
{
  return UpdateMOOSVariables(NewMail);
}

bool iOS5000::PublishData()
{
  return PublishFreshMOOSVariables();
}



bool iOS5000::OnConnectToServer()
{
  if (IsSimulateMode()) {
    //not much to do...
    RegisterMOOSVariables();
  } else {

  }

  return true;
}


///////////////////////////////////////////////////////////////////////////
// here we initialise the sensor, giving it start up values
bool iOS5000::InitialiseSensor()
{
  //We don't have any need to initialize the OS5000

  return true;

}

bool iOS5000::GetData()
{
  if (!IsSimulateMode()) {
    //here we actually access serial ports etc
    
    string sWhat;
    double dfWhen;

    if (m_Port.IsStreaming())
      {
	if (!m_Port.GetLatest(sWhat, dfWhen))
	  return false;
      }
    else
      {
	if (!m_Port.GetTelegram(sWhat, 0.5))
	  return false;
      }
    //MOOSTrace(sWhat+" ");
    ParseCRPTDString(sWhat);
  }
  else
    {
      //in simulated mode there is nothing to do..all data
      //arrives via comms.
    }

  return true;

}

// Function that parses the string that is coming from the Ocean Science 5000
//	Should be $C(compass data)P(pitch data)R(roll data)T(temp data)D(pressure? data)
//	 at a baudrate of 38400 or sometimes 19200
bool iOS5000::ParseCRPTDString(string &sCPRTDString)
{
	string sCopy = sCPRTDString;
	//OK so extract data...
	double dfTimeNow = MOOSTime();

	// Compass String
	string sWhat = MOOSChomp(sCPRTDString, "C");
	string sCompass = MOOSChomp(sCPRTDString, "P");
	//MOOSTrace("Head: " + sCompass + " ");
	SetMOOSVar("C", atof(sCompass.c_str()), dfTimeNow);

	// Pitch String
	string sPitch = MOOSChomp(sCPRTDString, "R");
	//MOOSTrace(sPitch);
	SetMOOSVar("P", atof(sPitch.c_str()), dfTimeNow);

	// Roll String
	string sRoll = MOOSChomp(sCPRTDString, "T");
	//MOOSTrace(sRoll);
	SetMOOSVar("R", atof(sRoll.c_str()), dfTimeNow);

	// Temperature string
	string sTemp = MOOSChomp(sCPRTDString, "D");
	//MOOSTrace(sTemp);
	SetMOOSVar("T", atof(sTemp.c_str()), dfTimeNow);
	return true;
}

