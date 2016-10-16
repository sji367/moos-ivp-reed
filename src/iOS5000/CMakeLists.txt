#--------------------------------------------------------
# The CMakeLists.txt for:                    	iOS5000
# Author(s):    				Sam Reed
#--------------------------------------------------------

FILE(GLOB SRC *.cpp)

ADD_EXECUTABLE(iOS5000 ${SRC})
   
TARGET_LINK_LIBRARIES(iOS5000
   ${MOOS_LIBRARIES}
   ${MOOSGeodesy_LIBRARIES}
   anrp_util
   mbutil
   m
   pthread)
