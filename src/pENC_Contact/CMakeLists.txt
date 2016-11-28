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


# GDAL
FIND_PACKAGE(GDAL REQUIRED)
include_directories(${GDAL_INCLUDE_DIRS})


ADD_EXECUTABLE(pENC_Contact ${SRC})

TARGET_LINK_LIBRARIES(pENC_Contact
   ${MOOS_LIBRARIES}
   ${GDAL_LIBRARY}
   ${MOOSGeodesy_LIBRARIES}
   mbutil
   m
   pthread)

