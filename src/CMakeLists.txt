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
ADD_SUBDIRECTORY(lib_AStar)
ADD_SUBDIRECTORY(lib_ENC_util)
ADD_SUBDIRECTORY(lib_ENC)
ADD_SUBDIRECTORY(lib_anrp_util)
ADD_SUBDIRECTORY(iZBoat)
ADD_SUBDIRECTORY(iGPS_MB1)
ADD_SUBDIRECTORY(iOS5000)
ADD_SUBDIRECTORY(pENC_Contact)
ADD_SUBDIRECTORY(pENC_Print)
ADD_SUBDIRECTORY(pENC_WPT_Check)
ADD_SUBDIRECTORY(pPub_Points)
ADD_SUBDIRECTORY(Test)
ADD_SUBDIRECTORY(UWTROC)

##############################################################################
#                           END of CMakeLists.txt
##############################################################################

