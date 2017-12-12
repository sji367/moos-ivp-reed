#--------------------------------------------------------
# The CMakeLists.txt for:              lib_behaviors-test
# Author(s):                                
#--------------------------------------------------------

# Set System Specific Libraries
if (${WIN32})
  # Windows Libraries
  SET(SYSTEM_LIBS
      )
else (${WIN32})
  # Linux and Apple Libraries
  SET(SYSTEM_LIBS
      m )
endif (${WIN32})


MACRO(ADD_BHV BHV_NAME)
  ADD_LIBRARY(${BHV_NAME} SHARED ${BHV_NAME}.cpp)
  TARGET_LINK_LIBRARIES(${BHV_NAME} 
    helmivp
    behaviors 
    ivpbuild 
    logic 
    ivpcore 
    bhvutil 
    mbutil 
    geometry 
    ${SYSTEM_LIBS} )
ENDMACRO(ADD_BHV)

#--------------------------------------------------------
#                                      	       BHV_OA_pnt
#--------------------------------------------------------
ADD_LIBRARY(BHV_OA_pnt SHARED 
   BHV_OA_pnt.cpp )
TARGET_LINK_LIBRARIES(BHV_OA_pnt
   helmivp
   behaviors 
   ivpbuild 
   logic 
   ivpcore 
   bhvutil 
   mbutil 
   geometry
   ENC_util
   ${SYSTEM_LIBS} )

#--------------------------------------------------------
#                                      	       BHV_polyOA
#--------------------------------------------------------
ADD_LIBRARY(BHV_polyOA SHARED 
   BHV_polyOA.cpp )
TARGET_LINK_LIBRARIES(BHV_polyOA
   helmivp
   behaviors 
   ivpbuild 
   logic 
   ivpcore 
   bhvutil 
   mbutil 
   geometry
   ENC_util
   ${SYSTEM_LIBS} )
