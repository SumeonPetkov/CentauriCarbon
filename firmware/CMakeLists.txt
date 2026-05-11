project(app)
cmake_minimum_required(VERSION 3.5)

option(ENABLE_MANUTEST "Enable compilation of manufacturing test" OFF)
set(CMAKE_C_COMPILER "arm-openwrt-linux-gcc")
set(CMAKE_CXX_COMPILER "arm-openwrt-linux-g++")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread -lrt -Wno-psabi")

add_definitions("-O3 -g")
add_definitions("-DQ_OS_LINUX")
add_definitions(
    -D_LARGEFILE64_SOURCE
    -D_FILE_OFFSET_BITS=64
    -D MG_ARCH=MG_ARCH_UNIX
)
add_definitions(-Wno-builtin-macro-redefined)
add_definitions(-DKVS_USE_OPENSSL)

if(ENABLE_MANUTEST)
    message(STATUS "Compiling with manufacturing test")
    aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/manu_test SRC_LIST)
    aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/manu_test/lib SRC_LIST)
    aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/manu_test/cli_cmds SRC_LIST)
    add_definitions(-DENABLE_MANUTEST=1)
else()
    message(STATUS "Manufacturing test disabled")
    add_definitions(-DENABLE_MANUTEST=0)
endif()
    

add_compile_options(-Wno-write-strings)
link_libraries(${CMAKE_CURRENT_SOURCE_DIR}/lib/ubToolenvLib.a) #添加静态库

aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/ SRC_LIST)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/api SRC_LIST)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/core SRC_LIST)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/core/allwinner SRC_LIST)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/core/klippy SRC_LIST)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/core/klippy/chelper SRC_LIST)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/core/klippy/extras SRC_LIST)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/core/klippy/helper SRC_LIST)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/core/klippy/adx_lis2dw12 SRC_LIST)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/core/klippy/kinematics SRC_LIST)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/devices SRC_LIST)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/utils SRC_LIST)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/ui SRC_LIST)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/ui/loader SRC_LIST)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/hl/devices SRC_LIST)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/hl/drivers SRC_LIST)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/hl/utils SRC_LIST)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/core/aw/ SRC_LIST)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/core/aw/librpbuf/ SRC_LIST)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/core/aw/librpmsg/ SRC_LIST)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/app/e100 SRC_LIST)

aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/service SRC_LIST)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/gui SRC_LIST)
file(GLOB_RECURSE LVGL_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/gui/lvgl/src/*.c)









include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/
    ${CMAKE_CURRENT_SOURCE_DIR}/api
    ${CMAKE_CURRENT_SOURCE_DIR}/core
    ${CMAKE_CURRENT_SOURCE_DIR}/core/allwinner
    ${CMAKE_CURRENT_SOURCE_DIR}/core/klippy
    ${CMAKE_CURRENT_SOURCE_DIR}/core/klippy/chelper
    ${CMAKE_CURRENT_SOURCE_DIR}/core/klippy/extras
    ${CMAKE_CURRENT_SOURCE_DIR}/core/klippy/helper
    ${CMAKE_CURRENT_SOURCE_DIR}/core/klippy/adx_lis2dw12
    ${CMAKE_CURRENT_SOURCE_DIR}/core/klippy/kinematics
    ${CMAKE_CURRENT_SOURCE_DIR}/devices
    ${CMAKE_CURRENT_SOURCE_DIR}/utils
    ${CMAKE_CURRENT_SOURCE_DIR}/app/e100


    ${CMAKE_CURRENT_SOURCE_DIR}/gui/
    ${CMAKE_CURRENT_SOURCE_DIR}/gui/lvgl
    ${CMAKE_CURRENT_SOURCE_DIR}/ui/
    ${CMAKE_CURRENT_SOURCE_DIR}/ui/loader


    ${CMAKE_CURRENT_SOURCE_DIR}/hl/devices
    ${CMAKE_CURRENT_SOURCE_DIR}/hl/drivers
    ${CMAKE_CURRENT_SOURCE_DIR}/hl/utils
    
    ${CMAKE_CURRENT_SOURCE_DIR}/include/freetype2
    ${CMAKE_CURRENT_SOURCE_DIR}/include/freetype2/freetype
    ${CMAKE_CURRENT_SOURCE_DIR}/include/ts
    ${CMAKE_CURRENT_SOURCE_DIR}/include/wpa_client
    ${CMAKE_CURRENT_SOURCE_DIR}/include/curl
    ${CMAKE_CURRENT_SOURCE_DIR}/include/nghttp2
    ${CMAKE_CURRENT_SOURCE_DIR}/include/uuid
    ${CMAKE_CURRENT_SOURCE_DIR}/include/zlib
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/core/aw/
    ${CMAKE_CURRENT_SOURCE_DIR}/core/aw/librpbuf
    ${CMAKE_CURRENT_SOURCE_DIR}/core/aw/librpmsg

    ${CMAKE_CURRENT_SOURCE_DIR}/lib/bz2
    ${CMAKE_CURRENT_SOURCE_DIR}/include/png
    ${CMAKE_CURRENT_SOURCE_DIR}/include/nghttp2
    ${CMAKE_CURRENT_SOURCE_DIR}/service

    ${CMAKE_CURRENT_SOURCE_DIR}/include/tasn1
    ${CMAKE_CURRENT_SOURCE_DIR}/include/hogweed
    ${CMAKE_CURRENT_SOURCE_DIR}/include/gmp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/opus
    ${CMAKE_CURRENT_SOURCE_DIR}/include/nettle
    ${CMAKE_CURRENT_SOURCE_DIR}/include/gnutls
    ${CMAKE_CURRENT_SOURCE_DIR}/include/x264
    ${CMAKE_CURRENT_SOURCE_DIR}/include/ffmpeg
if(ENABLE_MANUTEST)
    ${CMAKE_CURRENT_SOURCE_DIR}/manu_test
    ${CMAKE_CURRENT_SOURCE_DIR}/manu_test/lib
    ${CMAKE_CURRENT_SOURCE_DIR}/manu_test/cli_cmds
endif()
)

link_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/ts
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/freetype
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/wpa_client
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/openssl
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/curl
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/uuid
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/zlib
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/bz2
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/png
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/nghttp2
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/unistring
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/tasn1
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/hogweed
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/gmp
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/opus
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/nettle
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/gnutls
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/x264
	${CMAKE_CURRENT_SOURCE_DIR}/lib/alsa
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/ffmpeg
)

add_executable(${CMAKE_PROJECT_NAME} ${SRC_LIST} ${LVGL_SOURCES})   

target_link_libraries(${CMAKE_PROJECT_NAME}    
    ts
    dl
    freetype
    ssl
    crypto
    pthread
    curl
    uuid
    z
    wpa_client
    gmp
    tasn1
    hogweed
    nettle
    gnutls
    avcodec
    avdevice
    avfilter
    avformat
    avutil
    swresample
    swscale
    x264
	png
)
