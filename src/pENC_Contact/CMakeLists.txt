#--------------------------------------------------------
# The CMakeLists.txt for:                       pENC_Contact
# Author(s):                              Sam Reed
#--------------------------------------------------------

# FILE(GLOB SRC *.cpp)

SET(SRC
  ENC_Contact.cpp
  ENC_Contact_Info.cpp
  main.cpp
)


ADD_EXECUTABLE(pENC_Contact ${SRC})

TARGET_LINK_LIBRARIES(pENC_Contact
   ${MOOS_LIBRARIES}
   ${GDAL_LIBRARY}
   ${MOOSGeodesy_LIBRARIES}
   ${SYSTEM_LIBS}
   helmivp
   behaviors 
   ivpbuild 
   logic 
   ivpcore 
   bhvutil 
   mbutil 
   geometry
   m
   pthread)

