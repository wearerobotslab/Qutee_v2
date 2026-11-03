# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.20.0)
set(CMAKE_VERBOSE_MAKEFILE ON)

# Which application to build: the base robot (main/) plus one folder of apps/. Each application has its
# own build folder and binary, e.g. for the neural network controller:
#   idf.py -B build/nn -DQUTEE_APP=nn build      ->  build/nn/qutee_nn.bin
# (build.sh does this inside the qutee_idf container). The build folder remembers its application.
set(QUTEE_APPS qutee nn homeokinesis joint_reader)
set(QUTEE_APP "" CACHE STRING "Application to build: one of ${QUTEE_APPS}")
if(NOT QUTEE_APP IN_LIST QUTEE_APPS)
  message(FATAL_ERROR "Choose the application with -DQUTEE_APP=<app>, one of: ${QUTEE_APPS}. "
                      "For example: ./build.sh nn")
endif()

set (EXTRA_COMPONENT_DIRS "./components/micro_ros_espidf_component/.")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(qutee_${QUTEE_APP})
