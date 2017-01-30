#--------------------------------------------------------
# The CMakeLists.txt for:                   lib_anrp_util
# Author(s):                          Andrew Patrikalakis
#--------------------------------------------------------

FILE(GLOB SRC *.cpp *.c)

ADD_LIBRARY(ENC_geom ${BUILD_STATIC_OR_SHARED_LIBRARIES} ${SRC})

TARGET_LINK_LIBRARIES(ENC_geom
${MOOS_LIBRARIES})
