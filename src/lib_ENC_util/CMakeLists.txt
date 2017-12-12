#--------------------------------------------------------
# The CMakeLists.txt for:                    lib_ENC_util
# Author(s):                                     Sam Reed
#--------------------------------------------------------

cmake_minimum_required(VERSION 2.8.11)

# Boost and its components
find_package( Boost COMPONENTS system filesystem REQUIRED )

if ( NOT Boost_FOUND )

  message(STATUS "This project requires the Boost library, and will not be compiled.")

  return()  

endif()


FILE(GLOB SRC *.cpp *.c)

ADD_LIBRARY(ENC_util ${BUILD_STATIC_OR_SHARED_LIBRARIES} ${SRC})


TARGET_LINK_LIBRARIES(ENC_util
   ${GDAL_LIBRARY}
   ${MOOSGeodesy_LIBRARIES}
   ${MOOS_LIBRARIES}
   ${Boost_SYSTEM_LIBRARY}
   ${Boost_FILESYSTEM_LIBRARY}
   ASTAR
)

