#--------------------------------------------------------
# The CMakeLists.txt for:                       AStar
# Author(s):                              	Sam Reed
#--------------------------------------------------------

# Boost and its components
find_package( Boost COMPONENTS system filesystem REQUIRED )

if ( NOT Boost_FOUND )

  message(STATUS "This project requires the Boost library, and will not be compiled.")

  return()  

endif()

SET(SRC
  main.cpp
)

ADD_EXECUTABLE(A_Star ${SRC})

TARGET_LINK_LIBRARIES(A_Star
   ${GDAL_LIBRARY}
	ASTAR
)
