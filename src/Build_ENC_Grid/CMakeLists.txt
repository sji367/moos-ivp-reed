#--------------------------------------------------------
# The CMakeLists.txt for:                       iTest
# Author(s):                                 Sam Reed
#--------------------------------------------------------

SET(SRC
  ENC2Grid.cpp
)

ADD_EXECUTABLE(ENC2Grid ${SRC})

TARGET_LINK_LIBRARIES(ENC2Grid
   ${MOOS_LIBRARIES}
   ${GDAL_LIBRARY}
   ${MOOSGeodesy_LIBRARIES}
   apputil
   mbutil
   m
   pthread
   ENC_util)
