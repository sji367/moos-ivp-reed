#--------------------------------------------------------
# The CMakeLists.txt for:                       pAStar
# Author(s):                              Sam Reed
#--------------------------------------------------------

# FILE(GLOB SRC *.cpp)

SET(SRC
  AStar.cpp
  AStar_Info.cpp
  main.cpp
)



ADD_EXECUTABLE(pAStar ${SRC})

TARGET_LINK_LIBRARIES(pAStar
   ${MOOS_LIBRARIES}
   apputil
   mbutil
   m
   pthread
   ENC_util
   geometry

   mbutil 
   geometry
)

