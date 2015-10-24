cmake_minimum_required(VERSION 2.6)

project(glshim)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

link_directories(${CMAKE_LIBRARY_OUTPUT_DIRECTORY})

option(PANDORA "Set to ON if targeting an OpenPandora device" ${PANDORA})
option(BCMHOST "Set to ON if targeting an RPi(2) device" ${BCMHOST})
option(ODROID "Set to ON if targeting an ODroid device" ${ODROID})
option(ANDROID "Set to ON if targeting an Android device" ${ANDROID})

#select at least 1 plateform, PANDORA is default...
if(NOT PANDORA)
 if(NOT BCMHOST)
  if(NOT ODROID)
   if(NOT ANDROID)
    set(PANDORA ON CACHE BOOL "Set to ON if targeting an OpenPandora device" FORCE)
   endif()
  endif()
 endif()
endif()

# Raspberry PI
if(BCMHOST)
    include_directories(/opt/vc/include /opt/vc/include/interface/vcos/pthreads /opt/vc/include/interface/vmcs_host/linux)
    link_directories(/opt/vc/lib)
    add_definitions(-DBCMHOST)
endif()

if(PANDORA)
    add_definitions(-DPANDORA)
    add_definitions(-DTEXSTREAM)
endif()

if(ODROID)
    add_definitions(-DODROID)
endif()

if(ANDROID)
    add_definitions(-DANDROID)
    #add_definitions(-DBCMHOST)
    add_definitions(-include android_debug.h)
endif()

link_directories(${CMAKE_BINARY_DIR}/lib)
add_definitions(-g -std=gnu99 -funwind-tables -O3)

include_directories(include)
add_subdirectory(src)
