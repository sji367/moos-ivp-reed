#--------------------------------------------------------
# The CMakeLists.txt for:                       iGPS_UDP
# Author(s):    Paul Newman, modified Andrew Patrikalakis, Damian Manda
#--------------------------------------------------------

FILE(GLOB SRC *.cpp)

ADD_EXECUTABLE(iGPS ${SRC})
   
TARGET_LINK_LIBRARIES(iGPS
   ${MOOS_LIBRARIES}
   ${MOOSGeodesy_LIBRARIES}
   anrp_util
   mbutil
   m
   pthread)
