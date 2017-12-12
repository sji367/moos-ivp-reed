#--------------------------------------------------------
# The CMakeLists.txt for:                       pENC_Print
# Author(s):                              Sam Reed
#--------------------------------------------------------

# FILE(GLOB SRC *.cpp)

SET(SRC
  ENC_Print.cpp
  ENC_Print_Info.cpp
  main.cpp
)



ADD_EXECUTABLE(pENC_Print ${SRC})

TARGET_LINK_LIBRARIES(pENC_Print
   ${MOOS_LIBRARIES}
   ${GDAL_LIBRARY}	
   mbutil
   m
   pthread
   ENC_util)

