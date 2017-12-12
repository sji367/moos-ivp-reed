#--------------------------------------------------------
# The CMakeLists.txt for:                       iTest
# Author(s):                              Damian Manda
#--------------------------------------------------------

# CGAL and its components


# Boost and its components
find_package( Boost COMPONENTS system filesystem REQUIRED )

if ( NOT Boost_FOUND )

  message(STATUS "This project requires the Boost library, and will not be compiled.")

  return()  

endif()


SET(SRC
  UWTROC.cpp
)

ADD_EXECUTABLE(UWTROC ${SRC})

TARGET_LINK_LIBRARIES(UWTROC
   ${MOOS_LIBRARIES}
   ${GDAL_LIBRARY}
   apputil
   mbutil
   m
   pthread
   geometry
   ENC_util
   ${Boost_SYSTEM_LIBRARY}
   ${Boost_FILESYSTEM_LIBRARY}
)
