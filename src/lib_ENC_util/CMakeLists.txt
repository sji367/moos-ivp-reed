#--------------------------------------------------------
# The CMakeLists.txt for:                    lib_ENC_util
# Author(s):                                     Sam Reed
#--------------------------------------------------------

FILE(GLOB SRC *.cpp *.c)

ADD_LIBRARY(ENC_util ${BUILD_STATIC_OR_SHARED_LIBRARIES} ${SRC})

TARGET_LINK_LIBRARIES(ENC_util
   ${GDAL_LIBRARY}
   ${MOOSGeodesy_LIBRARIES}
   ${MOOS_LIBRARIES}
)
