# Install script for directory: /home/sreed/moos-ivp/moos-ivp-reed/src

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "None")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/sreed/moos-ivp/moos-ivp-reed/build/src/lib_AStar/cmake_install.cmake")
  include("/home/sreed/moos-ivp/moos-ivp-reed/build/src/lib_ENC_util/cmake_install.cmake")
  include("/home/sreed/moos-ivp/moos-ivp-reed/build/src/lib_ENC/cmake_install.cmake")
  include("/home/sreed/moos-ivp/moos-ivp-reed/build/src/lib_anrp_util/cmake_install.cmake")
  include("/home/sreed/moos-ivp/moos-ivp-reed/build/src/iZBoat/cmake_install.cmake")
  include("/home/sreed/moos-ivp/moos-ivp-reed/build/src/iGPS_MB1/cmake_install.cmake")
  include("/home/sreed/moos-ivp/moos-ivp-reed/build/src/iOS5000/cmake_install.cmake")
  include("/home/sreed/moos-ivp/moos-ivp-reed/build/src/pENC_Contact/cmake_install.cmake")
  include("/home/sreed/moos-ivp/moos-ivp-reed/build/src/pENC_Print/cmake_install.cmake")
  include("/home/sreed/moos-ivp/moos-ivp-reed/build/src/pENC_WPT_Check/cmake_install.cmake")
  include("/home/sreed/moos-ivp/moos-ivp-reed/build/src/pPub_Points/cmake_install.cmake")
  include("/home/sreed/moos-ivp/moos-ivp-reed/build/src/Test/cmake_install.cmake")
  include("/home/sreed/moos-ivp/moos-ivp-reed/build/src/UWTROC/cmake_install.cmake")

endif()

