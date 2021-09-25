# cppzmq cmake module
#
# The following import targets are created
#
# ::
#
#   cppzmq-static
#   cppzmq
#
# This module sets the following variables in your project::
#
# cppzmq_FOUND - true if cppzmq found on the system
# cppzmq_INCLUDE_DIR - the directory containing cppzmq headers
# cppzmq_LIBRARY - the ZeroMQ library for dynamic linking
# cppzmq_STATIC_LIBRARY - the ZeroMQ library for static linking


####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was cppzmqConfig.cmake.in                            ########

get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../" ABSOLUTE)

macro(set_and_check _var _file)
  set(${_var} "${_file}")
  if(NOT EXISTS "${_file}")
    message(FATAL_ERROR "File or directory ${_file} referenced by variable ${_var} does not exist !")
  endif()
endmacro()

macro(check_required_components _NAME)
  foreach(comp ${${_NAME}_FIND_COMPONENTS})
    if(NOT ${_NAME}_${comp}_FOUND)
      if(${_NAME}_FIND_REQUIRED_${comp})
        set(${_NAME}_FOUND FALSE)
      endif()
    endif()
  endforeach()
endmacro()

####################################################################################

include(CMakeFindDependencyMacro)
find_package(ZeroMQ QUIET)

# libzmq autotools install: fallback to pkg-config
if(NOT ZeroMQ_FOUND)
    list (APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/libzmq-pkg-config)
    find_package(ZeroMQ REQUIRED)
endif()

if(NOT ZeroMQ_FOUND)
    message(FATAL_ERROR "ZeroMQ was NOT found!")
endif()

if(NOT TARGET cppzmq)
    include("${CMAKE_CURRENT_LIST_DIR}/cppzmqTargets.cmake")
    get_target_property(cppzmq_INCLUDE_DIR cppzmq INTERFACE_INCLUDE_DIRECTORIES)
endif()

