# **********************************************************************************************
# ************************* MAIN PROPERTIES (YOU NEED TO EDIT THEM!!) **************************
# **********************************************************************************************

#the name of the project
set(THEPROJECT_NAME "pathfinding-map-perturbator")

#the version of the project
set(THEPROJECT_VERSION 1.0)

#what will be prodiced: either EXE (executable); SO (shared library) AO (static library)
#Can be overriden by using "cmake -DU_LIBRARY_TYPE:STRING=<newvalue>" command
set(THEPROJECT_OUTPUT "EXE")

#a spaced separated list of shared libraries names that will be used when linking the main project. 
#Each library needs to be installed on the system. Each library should be declared as a quoted string
set(THEPROJECT_REQUIRED_SHARED_LIBRARIES "cpp-utils" "pathfinding-utils" "pathfinding-map-perturbator-utils" "boost_filesystem" "dl")

#Represents the library official name of an entry in THEPROJECT_REQUIRED_SHARED_LIBRARIES.
#This has been done because sometimes a library soname is not the library common nam e (e.g., "m" is "math")
#The variable is a spaced separated list of names. The i-th element in THEPROJECT_REQUIRED_SHARED_LIBRARIES_NAMES
#The cell is empty ig the library soname is the same of the library official name.
#is the name of the i-th library in THEPROJECT_REQUIRED_SHARED_LIBRARIES
set(THEPROJECT_REQUIRED_SHARED_LIBRARIES_NAMES "cpp-utils" "pathfinding-utils" "pathfinding-map-perturbator-utils" "boost_filesystem" "dl")

#Represents the package name we need to install with "apt-get" software of an entry in THEPROJECT_REQUIRED_SHARED_LIBRARIES.
#This has been done because in this way this script can automatically install dependency if needed (this feature has not been done yet though)
#For example if the library is "metis" here we should have "metis" because "sudo apt-get install metis" will install metis
#The variable is a spaced separated list of names. The i-th element in THEPROJECT_REQUIRED_SHARED_LIBRARIES_NAMES.
#An empty string if the library cannot be installed via apt-get.
#is the name of the i-th library in THEPROJECT_REQUIRED_SHARED_LIBRARIES
set(THEPROJECT_REQUIRED_SHARED_LIBRARIES_APTGET "")

#a spaced separated list of additional shared libraries that will be used when linking the test application. Each library needs to be installed
#ignore it if you put "THEPROJECT_TEST_ENABLE_TEST_COMPILATION" to "false" 
set(THEPROJECT_TEST_ADDITIONAL_SHARED_LIBRARIES "")

#Represents the library official name of an entry in THEPROJECT_TEST_ADDITIONAL_SHARED_LIBRARIES.
#This has been done because sometimes a library soname is not the library common nam e (e.g., "m" is "math")
#The variable is a spaced separated list of names. The i-th element in THEPROJECT_TEST_ADDITIONAL_SHARED_LIBRARIES_NAMES
#is the name of the i-th library in THEPROJECT_TEST_ADDITIONAL_SHARED_LIBRARIES
set(THEPROJECT_TEST_ADDITIONAL_SHARED_LIBRARIES_NAMES "")

#Represents the package name we need to install with "apt-get" software of an entry in THEPROJECT_TEST_ADDITIONAL_SHARED_LIBRARIES.
#This has been done because in this way this script can automatically install dependency if needed (this feature has not been done yet though)
#For example if the library is "metis" here we should have "metis" because "sudo apt-get install metis" will install metis
#The variable is a spaced separated list of names. The i-th element in THEPROJECT_TEST_ADDITIONAL_SHARED_LIBRARIES_NAMES.
#An empty string if the library cannot be installed via apt-get
#is the name of the i-th library in THEPROJECT_TEST_ADDITIONAL_SHARED_LIBRARIES
set(THEPROJECT_TEST_ADDITIONAL_SHARED_LIBRARIES_APTGET "")

#true if you want to compile the all the tests inside src/test/c src/test/include.
#values: "true", "false"
set(THEPROJECT_TEST_ENABLE_TEST_COMPILATION "true")

#If you're building a library, use this variable to enable or disable the -fPIC flag. Ignored if not building library.
#turning on will allow multiple process to share the same library object code but it will reduce performances.
#By turning off every process using the library will have its own copy of the library code, but it will increase performances.
#Can be overriden by using "cmake -DU_FPIC:STRING=<newvalue>" command  
set(THEPROJECT_POSITION_INDEPENDENT_CODE "true")

#The log level of the debug build for this project.
#Can be overriden by using "cmake -U_DEBUG_LOG_LEVEL:STRING=<newvalue>" command
# Values go from 0(debug) till 7 (critical). Values are:
# 0: DEBUG
# 1: FINEST
# 2: FINER
# 3: FINE
# 4: <undefined>
# 5: INFO
# 6: WARNING
# 7: ERROR
# 8 CRITICAL
set(THEPROJECT_DEBUG_LOG_LEVEL "3")

#put true if you have changed something inside this cmake standard building process; false otherwise
set(STANDARD_CMAKE_FILE_ALTERED "false")

#If you have altered the standard CMAKE file standard process, consider explaining in this variable what have you changed to help future maintainers!
#The variable is ignored if "STANDARD_CMAKE_FILE_ALTERED" is false
set(CMAKE_FILE_ALTERED_COMMAND "")

#Represents the version of the building process version. You can use this value to understand what this cmake building process can and can't do
#For example in building processes before the "1.0" "sudo make install" of exectuables wasn't supported.
# - 1.0: first version
# - 1.1: sudo make install for static libraries as well
# - 1.2: cmake version log change
# - 1.3: added position independent code, cmake command line arguments
# - 1.4: "make install/uninstall" works without root access; Use U_INSTALL_DIRECTORY to alter installation directory; refactored  messages
# - 1.5: ease to tweak log in cmake
# - 1.6: fmax-errors=1 isntead of -Wfatal-errors
# - 1.7: add done message at the end of successful compilation
# - 1.8: added Catch framework in test, log default to 3, improve documentation, add library names and apt-get
set(CMAKE_FILE_BUILDING_PROCESS_VERSION "1.8")

#The place where all the stuff created by "sudo make install" will be positioned"
#Leave it if you don't want to change the behaviour. Can be overriden by U_INSTALL_DIRECTORY. For example: "-DU_INSTALL_DIRECTORY=~/usr"
#(no end slash!).
set(THEPROJECT_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")




















































# **********************************************************************************************
# ************************* CMAKE SCRIPT (DO NOT EDIT!!) ***************************************
# **********************************************************************************************

# ******************** MACRO DEFINITIONS ***************************

# Given an absolute path of a directory, fetch all the subdirectories
#
# @code
# SUBDIRLIST(OUTPUT "/home/koldar/example")
# @endcode
#
# @param result the list you want to populate with subdirectories
# @param absolutepathdir absolute path of the roto directory. No "/" should be put at the end of the directory
# return list fo absolute paths of the subdirectories. root directory **not** included!
MACRO(SUBDIRLIST result absolutepathdir)
  FILE(GLOB_RECURSE children LIST_DIRECTORIES true RELATIVE ${absolutepathdir} ${absolutepathdir}/*)
  SET(dirlist "")
  FOREACH(child ${children})
    IF(IS_DIRECTORY ${absolutepathdir}/${child})
      LIST(APPEND dirlist ${absolutepathdir}/${child})
    ENDIF()
  ENDFOREACH()
  SET(${result} ${dirlist})
ENDMACRO()

# ******************** CHECK CONSTRAINTS ***************************

cmake_minimum_required(VERSION 2.8.7)
project(${THEPROJECT_NAME})

# ******************** IMPORTANT INCLUDES **************************

# ******************** OVERRIDING  CHANGABLE VARIABLES ************

SET(U_FPIC "" CACHE STRING "true to enable PIC. False to enable Relocation")
SET(U_LIBRARY_TYPE "" CACHE STRING "SO for shared library, AO for static library")
SET(U_INSTALL_DIRECTORY "" CACHE STRING "The place where everything 'sudo make install' is positioned")
SET(U_DEBUG_LOG_LEVEL "" CACHE STRING "value going from 0(debug) to 7 (critical) for logging (only for debug)")

if (NOT ${U_FPIC} STREQUAL "")
    set(THEPROJECT_POSITION_INDEPENDENT_CODE ${U_FPIC})
    message(STATUS "${BoldYellow}changing FPIC to ${THEPROJECT_POSITION_INDEPENDENT_CODE}${ColorReset}")
endif()

if (NOT ${U_LIBRARY_TYPE} STREQUAL "")
    set(THEPROJECT_OUTPUT ${U_LIBRARY_TYPE})
    message(STATUS "${BoldYellow}changing library type to ${THEPROJECT_OUTPUT}${ColorReset}")
endif()

if (NOT ${U_INSTALL_DIRECTORY} STREQUAL "")
    set(THEPROJECT_INSTALL_PREFIX ${U_INSTALL_DIRECTORY})
    message(STATUS "${BoldYellow}changing install directory to ${THEPROJECT_INSTALL_PREFIX}${ColorReset}")
endif()

if (NOT ${U_DEBUG_LOG_LEVEL} STREQUAL "")
    set(THEPROJECT_DEBUG_LOG_LEVEL ${U_DEBUG_LOG_LEVEL})
    message(STATUS "${BoldYellow}changing debug log level to ${THEPROJECT_DEBUG_LOG_LEVEL}${ColorReset}")
endif()

# ************************ SET DEFINITIVE VARIABLES ***************************

#the place where everything will be install into
SET(CMAKE_INSTALL_PREFIX ${THEPROJECT_INSTALL_PREFIX})
#make the make file always verbose (https://stackoverflow.com/questions/4808303/making-cmake-print-commands-before-executing)
set(CMAKE_VERBOSE_MAKEFILE on)

get_filename_component(PARENTDIR ${CMAKE_BINARY_DIR} NAME)

# create string constants (https://stackoverflow.com/a/19578320/1887602)
if(NOT WIN32)
    string(ASCII 27 Esc)
    set(ColorReset "${Esc}[m")
    set(BoldRed     "${Esc}[1;31m")
    set(BoldCyan    "${Esc}[1;36m")
    set(BoldYellow  "${Esc}[1;33m")
endif()

# ******************** BUILDING SUMMARY ***************************

message(STATUS "${BoldYellow}You should call cmake when you are in build/Debug or in build/Release. Perform 'mkdir -p build/Debug; cd build/Debug; cmake ../..'${ColorReset}")
message(STATUS "${BoldYellow}This cmake generates a building process with version ${CMAKE_FILE_BUILDING_PROCESS_VERSION}${ColorReset}")
if (${STANDARD_CMAKE_FILE_ALTERED} STREQUAL "true")
    message(STATUS "${BoldYellow}This cmake building process has been altered from the standard one! This means you need to look at the CMakeLists.txt file as well to understand what has been added!${ColorReset}")
    message(STATUS "${BoldYellow}Here's an explanation of the changes:${ColorReset}\n\n")
    message(STATUS "${BoldYellow}${CMAKE_FILE_ALTERED_COMMAND}\n\n${ColorReset}")
endif()

message(STATUS "${BoldCyan}cmake is working in directory ${PARENTDIR}${ColorReset}")
message(STATUS "${BoldCyan}cmake will build your application in ${CMAKE_BINARY_DIR}${ColorReset}")
message(STATUS "${BoldCyan}cmake will 'sudo make install' your application in ${CMAKE_INSTALL_PREFIX}${ColorReset}") 
if(${THEPROJECT_OUTPUT} STREQUAL "SO")
    message(STATUS "${BoldCyan}We will build a shared library${ColorReset}")
elseif(${THEPROJECT_OUTPUT} STREQUAL "AO")
    message(STATUS "${BoldCyan}We will build a static library${ColorReset}")
elseif(${THEPROJECT_OUTPUT} STREQUAL "EXE")
    message(STATUS "${BoldCyan}We will build an executable${ColorReset}")
endif()
message(STATUS "${BoldCyan}log level for debug build set to ${THEPROJECT_DEBUG_LOG_LEVEL}${ColorReset}")


# ******************** BUILDING OPTIONS ***************************

if(PARENTDIR STREQUAL "Release")
    message(STATUS "${BoldCyan}Building Release!${ColorReset}")
    #totally disable log.hpp
    set(CMAKE_BUILD_TYPE "Release")
    add_definitions(-DQUICK_LOG=7)
endif()

if(PARENTDIR STREQUAL "Debug")
    message(STATUS "${BoldCyan}Building Debug!${ColorReset}")
    set(CMAKE_BUILD_TYPE "Debug")
    #-fno-stack-protector: to debug stack smashing (https://stackoverflow.com/a/1347464/1887602)
    add_definitions(-DQUICK_LOG=${THEPROJECT_DEBUG_LOG_LEVEL} -DDEBUG -fno-stack-protector )
endif(PARENTDIR STREQUAL "Debug")
#add common definitions
#I perfer fmax-errors=1 instead of -Wfatal-errors becuase the second won't show the compiler notes of the error, but the first will
add_definitions(-fmax-errors=1 -std=c++11 -Werror)

# ****************** SUB DIRECTORIES *************************
add_subdirectory(src/main/cpp)
if(${THEPROJECT_TEST_ENABLE_TEST_COMPILATION} STREQUAL "true")
    add_subdirectory(src/test/cpp)
endif(${THEPROJECT_TEST_ENABLE_TEST_COMPILATION} STREQUAL "true")

