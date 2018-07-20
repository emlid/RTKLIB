cmake_minimum_required(VERSION 3.5)
project(rtklib)

set(CMAKE_C_COMPILER g++)
set(CMAKE_CXX_COMPILER g++)

set(CMAKE_CONFIGURATION_TYPES "Release;Prerelease;Debug" CACHE STRING "" FORCE)
add_compile_options(
    -std=gnu++1z
    -pedantic
    -Wall
    -fno-strict-overflow
    -Wno-error=unused-but-set-variable
    -Wno-error=unused-function
    -Wno-error=unused-result
)
add_definitions(-DTRACE -DENAGLO -DENAQZS -DENAGAL -DENACMP -DENAIRN -DNFREQ=3 -DSVR_REUSEADDR)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING
      "Choose the type of build, options are: Debug Release Prerelease."
      FORCE)
endif(NOT CMAKE_BUILD_TYPE)

set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")
if ("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
    add_definitions(-DNDEBUG)
endif()

set(CMAKE_C_FLAGS_PRERELEASE "${CMAKE_C_FLAGS} -O3" CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS_PRERELEASE "${CMAKE_CXX_FLAGS} -O3" CACHE STRING "" FORCE)

mark_as_advanced(
    CMAKE_C_FLAGS_PRERELEASE
    CMAKE_CXX_FLAGS_PRERELEASE)

set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -O0 -fno-omit-frame-pointer -fsanitize=address -g")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0 -fno-omit-frame-pointer -fsanitize=address -g")

set(CMAKE_BUILD_TYPE "${CMAKE_BUILD_TYPE}" CACHE STRING
    "Choose the type of build, options are: Debug Release Prerelease."
    FORCE)

set(rtklib_SOURCE_FILES
    ${rtklib_SOURCE_DIR}/src/rcv/binex.c
    ${rtklib_SOURCE_DIR}/src/rcv/cmr.c
    ${rtklib_SOURCE_DIR}/src/rcv/crescent.c
    ${rtklib_SOURCE_DIR}/src/rcv/gw10.c
    ${rtklib_SOURCE_DIR}/src/rcv/javad.c
    ${rtklib_SOURCE_DIR}/src/rcv/novatel.c
    ${rtklib_SOURCE_DIR}/src/rcv/nvs.c
    ${rtklib_SOURCE_DIR}/src/rcv/rcvlex.c
    ${rtklib_SOURCE_DIR}/src/rcv/rt17.c
    ${rtklib_SOURCE_DIR}/src/rcv/septentrio.c
    ${rtklib_SOURCE_DIR}/src/rcv/skytraq.c
    ${rtklib_SOURCE_DIR}/src/rcv/ss2.c
    ${rtklib_SOURCE_DIR}/src/rcv/tersus.c
    ${rtklib_SOURCE_DIR}/src/rcv/ublox.c

    ${rtklib_SOURCE_DIR}/src/convgpx.c
    ${rtklib_SOURCE_DIR}/src/convkml.c
    ${rtklib_SOURCE_DIR}/src/convrnx.c
    ${rtklib_SOURCE_DIR}/src/datum.c
    ${rtklib_SOURCE_DIR}/src/download.c
    ${rtklib_SOURCE_DIR}/src/ephemeris.c
    ${rtklib_SOURCE_DIR}/src/erb.c
    ${rtklib_SOURCE_DIR}/src/geoid.c
    ${rtklib_SOURCE_DIR}/src/geoid.h
    ${rtklib_SOURCE_DIR}/src/gis.c
    ${rtklib_SOURCE_DIR}/src/ionex.c
    ${rtklib_SOURCE_DIR}/src/lambda.c
    ${rtklib_SOURCE_DIR}/src/options.c
    ${rtklib_SOURCE_DIR}/src/pntpos.c
    ${rtklib_SOURCE_DIR}/src/postpos.c
    ${rtklib_SOURCE_DIR}/src/ppp_ar.c
    ${rtklib_SOURCE_DIR}/src/ppp.c
    ${rtklib_SOURCE_DIR}/src/ppp_corr.c
    ${rtklib_SOURCE_DIR}/src/preceph.c
    ${rtklib_SOURCE_DIR}/src/qzslex.c
    ${rtklib_SOURCE_DIR}/src/rcv
    ${rtklib_SOURCE_DIR}/src/rcvraw.c
    ${rtklib_SOURCE_DIR}/src/rinex.c
    ${rtklib_SOURCE_DIR}/src/rtcm2.c
    ${rtklib_SOURCE_DIR}/src/rtcm3.c
    ${rtklib_SOURCE_DIR}/src/rtcm3e.c
    ${rtklib_SOURCE_DIR}/src/rtcm.c
    ${rtklib_SOURCE_DIR}/src/rtkcmn.c
    ${rtklib_SOURCE_DIR}/src/rtkpos.c
    ${rtklib_SOURCE_DIR}/src/rtksvr.c
    ${rtklib_SOURCE_DIR}/src/sbas.c
    ${rtklib_SOURCE_DIR}/src/solution.c
    ${rtklib_SOURCE_DIR}/src/stream.c
    ${rtklib_SOURCE_DIR}/src/streamsvr.c
    ${rtklib_SOURCE_DIR}/src/tides.c
    ${rtklib_SOURCE_DIR}/src/tle.c
)

include_directories(${rtklib_SOURCE_DIR}/src)
add_library(librtk ${rtklib_SOURCE_FILES})

set(convbin_SOURCE_FILES
    ${rtklib_SOURCE_DIR}/app/convbin/convbin.c
)
set(rtkrcv_SOURCE_FILES
    ${rtklib_SOURCE_DIR}/app/rtkrcv/vt.c
    ${rtklib_SOURCE_DIR}/app/rtkrcv/rtkrcv.c
)
include_directories(${rtklib_SOURCE_DIR}/app/rtkrcv)

set(rnx2rtkp_SOURCE_FILES
    ${rtklib_SOURCE_DIR}/app/rnx2rtkp/rnx2rtkp.c
)

set(str2str_SOURCE_FILES
    ${rtklib_SOURCE_DIR}/app/str2str/str2str.c
)

add_executable(convbin ${convbin_SOURCE_FILES})
target_link_libraries(convbin librtk pthread)
target_link_libraries(convbin debug asan)

add_executable(rtkrcv ${rtkrcv_SOURCE_FILES})
target_link_libraries(rtkrcv librtk pthread)
target_link_libraries(rtkrcv debug asan)

add_executable(rnx2rtkp ${rnx2rtkp_SOURCE_FILES})
target_link_libraries(rnx2rtkp librtk pthread)
target_link_libraries(rnx2rtkp debug asan)

add_executable(str2str ${str2str_SOURCE_FILES})
target_link_libraries(str2str librtk pthread)
target_link_libraries(str2str debug asan)

install(TARGETS str2str rnx2rtkp convbin rtkrcv DESTINATION /usr/bin)
