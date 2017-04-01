#--------------------------------------------------------
# The CMakeLists.txt for:                       iTest
# Author(s):                              Damian Manda
#--------------------------------------------------------

SET(SRC
  t.cpp
)

ADD_EXECUTABLE(iTest ${SRC})

TARGET_LINK_LIBRARIES(iTest
   ${MOOS_LIBRARIES}
   ${GDAL_LIBRARY}
   apputil
   mbutil
   m
   pthread
   ENC_util)
