#--------------------------------------------------------
# The CMakeLists.txt for:                       pENC_WPT_Check
# Author(s):                              Sam Reed
#--------------------------------------------------------

# FILE(GLOB SRC *.cpp)

SET(SRC
  ENC_WPT_Check.cpp
  ENC_WPT_Check_Info.cpp
  main.cpp
)



ADD_EXECUTABLE(pENC_WPT_Check ${SRC})

TARGET_LINK_LIBRARIES(pENC_WPT_Check
   ${MOOS_LIBRARIES}
   ${GDAL_LIBRARY}
   mbutil
   m
   pthread
   ENC_util)

