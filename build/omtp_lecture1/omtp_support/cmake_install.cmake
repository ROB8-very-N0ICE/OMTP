# Install script for directory: /home/wizard/Documents/Escola/AAU/Master/8_semester/Object_manipulation_and_task_planning/OMTP/src/omtp_lecture1/omtp_support

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/wizard/Documents/Escola/AAU/Master/8_semester/Object_manipulation_and_task_planning/OMTP/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
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

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/wizard/Documents/Escola/AAU/Master/8_semester/Object_manipulation_and_task_planning/OMTP/build/omtp_lecture1/omtp_support/catkin_generated/installspace/omtp_support.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/omtp_support/cmake" TYPE FILE FILES
    "/home/wizard/Documents/Escola/AAU/Master/8_semester/Object_manipulation_and_task_planning/OMTP/build/omtp_lecture1/omtp_support/catkin_generated/installspace/omtp_supportConfig.cmake"
    "/home/wizard/Documents/Escola/AAU/Master/8_semester/Object_manipulation_and_task_planning/OMTP/build/omtp_lecture1/omtp_support/catkin_generated/installspace/omtp_supportConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/omtp_support" TYPE FILE FILES "/home/wizard/Documents/Escola/AAU/Master/8_semester/Object_manipulation_and_task_planning/OMTP/src/omtp_lecture1/omtp_support/package.xml")
endif()

