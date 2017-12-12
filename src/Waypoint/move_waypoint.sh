#!/bin/bash

# Rename the current waypoint files
cp ../../../ivp/src/lib_behaviors-marine/BHV_Waypoint.cpp ../../../ivp/src/lib_behaviors-marine/BHV_Waypoint.cpp.bak 
cp ../../../ivp/src/lib_behaviors-marine/BHV_Waypoint.h ../../../ivp/src/lib_behaviors-marine/BHV_Waypoint.h.bak
cp ../../../ivp/src/lib_bhvutil/WaypointEngine.cpp ../../../ivp/src/lib_bhvutil/WaypointEngine.cpp.bak
cp ../../../ivp/src/lib_bhvutil/WaypointEngine.h ../../../ivp/src/lib_bhvutil/WaypointEngine.h.bak

# Move the modified Waypoint files to the correct location
cp BHV_Waypoint.cpp ../../../ivp/src/lib_behaviors-marine/BHV_Waypoint.cpp 
cp BHV_Waypoint.h ../../../ivp/src/lib_behaviors-marine/BHV_Waypoint.h
cp WaypointEngine.cpp ../../../ivp/src/lib_bhvutil/WaypointEngine.cpp
cp WaypointEngine.h ../../../ivp/src/lib_bhvutil/WaypointEngine.h

