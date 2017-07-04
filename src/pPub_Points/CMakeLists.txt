#--------------------------------------------------------
# The CMakeLists.txt for:                       pPub_Points
# Author(s):                              
#--------------------------------------------------------

# FILE(GLOB SRC *.cpp)

SET(SRC
  Pub_Points.cpp
  Pub_Points_Info.cpp
  main.cpp
)



ADD_EXECUTABLE(pPub_Points ${SRC})

TARGET_LINK_LIBRARIES(pPub_Points
   ${MOOS_LIBRARIES}
   mbutil
   m
   pthread)

