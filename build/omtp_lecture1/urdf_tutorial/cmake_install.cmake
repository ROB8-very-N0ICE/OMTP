# Install script for directory: /home/wizard/Documents/Escola/AAU/Master/8_semester/Object_manipulation_and_task_planning/OMTP/src/omtp_lecture1/urdf_tutorial

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/wizard/Documents/Escola/AAU/Master/8_semester/Object_manipulation_and_task_planning/OMTP/build/omtp_lecture1/urdf_tutorial/catkin_generated/installspace/urdf_tutorial.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/urdf_tutorial/cmake" TYPE FILE FILES
    "/home/wizard/Documents/Escola/AAU/Master/8_semester/Object_manipulation_and_task_planning/OMTP/build/omtp_lecture1/urdf_tutorial/catkin_generated/installspace/urdf_tutorialConfig.cmake"
    "/home/wizard/Documents/Escola/AAU/Master/8_semester/Object_manipulation_and_task_planning/OMTP/build/omtp_lecture1/urdf_tutorial/catkin_generated/installspace/urdf_tutorialConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/urdf_tutorial" TYPE FILE FILES "/home/wizard/Documents/Escola/AAU/Master/8_semester/Object_manipulation_and_task_planning/OMTP/src/omtp_lecture1/urdf_tutorial/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/urdf_tutorial" TYPE DIRECTORY FILES
    "/home/wizard/Documents/Escola/AAU/Master/8_semester/Object_manipulation_and_task_planning/OMTP/src/omtp_lecture1/urdf_tutorial/images"
    "/home/wizard/Documents/Escola/AAU/Master/8_semester/Object_manipulation_and_task_planning/OMTP/src/omtp_lecture1/urdf_tutorial/meshes"
    "/home/wizard/Documents/Escola/AAU/Master/8_semester/Object_manipulation_and_task_planning/OMTP/src/omtp_lecture1/urdf_tutorial/launch"
    "/home/wizard/Documents/Escola/AAU/Master/8_semester/Object_manipulation_and_task_planning/OMTP/src/omtp_lecture1/urdf_tutorial/rviz"
    "/home/wizard/Documents/Escola/AAU/Master/8_semester/Object_manipulation_and_task_planning/OMTP/src/omtp_lecture1/urdf_tutorial/urdf"
    )
endif()

