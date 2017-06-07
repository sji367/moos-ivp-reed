##############################################################################
# FILE:        moos-ivp-extend/src/CMakeLists.txt
# DATE:        2010/09/07
# DESCRIPTION: CMakeLists.txt file for the moos-ivp-extend source directory
##############################################################################


#============================================================================
# Add the libraries in the current directory to the include path
#============================================================================
FILE(GLOB LOCAL_LIBRARY_DIRS ./lib_*)
INCLUDE_DIRECTORIES(${LOCAL_LIBRARY_DIRS})

#============================================================================
# List the subdirectories to build...
#============================================================================
ADD_SUBDIRECTORY(lib_ENC_util)
ADD_SUBDIRECTORY(lib_ENC)
ADD_SUBDIRECTORY(lib_behaviors-test)
ADD_SUBDIRECTORY(lib_anrp_util)
ADD_SUBDIRECTORY(pAStar)
ADD_SUBDIRECTORY(iZBoat)
ADD_SUBDIRECTORY(iGPS_MB1)
ADD_SUBDIRECTORY(iOS5000)
ADD_SUBDIRECTORY(pENC_Contact)
ADD_SUBDIRECTORY(Build_ENC_Grid)
ADD_SUBDIRECTORY(Test)
ADD_SUBDIRECTORY(pPub_Points)
ADD_SUBDIRECTORY(GridENC)

##############################################################################
#                           END of CMakeLists.txt
##############################################################################

