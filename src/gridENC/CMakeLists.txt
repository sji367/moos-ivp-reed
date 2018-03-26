#--------------------------------------------------------
# The CMakeLists.txt for:                       ENC_Grid
# Author(s):                              	Sam Reed
#--------------------------------------------------------

# Boost and its components
find_package( Boost COMPONENTS system filesystem REQUIRED )

if ( NOT Boost_FOUND )

  message(STATUS "This project requires the Boost library, and will not be compiled.")

  return()  

endif()


SET(SRC
  gridENC.cpp
  main.cpp
)

ADD_EXECUTABLE(gridENC ${SRC})

TARGET_LINK_LIBRARIES(gridENC
   ${GDAL_LIBRARY}
   ENC_util
	ASTAR
   ${Boost_SYSTEM_LIBRARY}
   ${Boost_FILESYSTEM_LIBRARY}
)
