ACLOCAL_AMFLAGS = -I m4

## build libcamcli ##
lib_LTLIBRARIES = libcamcli.la

include_HEADERS = inc/CameraClient.hpp
include_HEADERS += inc/CameraServer.hpp

libcamcli_la_CXXFLAGS  = -fPIC

libcamcli_la_CXXFLAGS += -I src
libcamcli_la_CXXFLAGS += -I inc
libcamcli_la_CXXFLAGS += -I ../QCamera2/HAL
libcamcli_la_CXXFLAGS += -I ../QCamera2/util
libcamcli_la_CXXFLAGS += -I ../libcamera/inc
libcamcli_la_CXXFLAGS += -I ../libcamera/src


libcamcli_la_SOURCES  = src/CameraClient.cpp
libcamcli_la_SOURCES += src/CameraClientUtil.cpp

libcamcli_la_LDFLAGS = -shared
libcamcli_la_LDFLAGS += -ldl -lrt -lcamparams

## build cameraserver ##

bin_PROGRAMS = cameraserver server_test


cameraserver_CXXFLAGS  = -I inc
cameraserver_CXXFLAGS += -I src
cameraserver_CXXFLAGS += -I ../QCamera2/HAL
cameraserver_CXXFLAGS += -I ../libcamera/inc
cameraserver_CXXFLAGS += -I ../libcamera/src

cameraserver_SOURCES = src/CameraServer.cpp

cameraserver_LDADD = -lpthread -lcamparams libcamcli.la ../libcamera/libcamera.la


## build cameraserver ##

server_test_CXXFLAGS  = -I inc
server_test_CXXFLAGS += -I src
server_test_CXXFLAGS += -I ../libcamera/inc
server_test_CXXFLAGS += -I ../libcamera/src
server_test_CXXFLAGS += -I ../QCamera2/HAL
server_test_CXXFLAGS += -I ../libcamera/inc
server_test_CXXFLAGS += -I ../mm-image-codec/qomx_core
server_test_CXXFLAGS += -I ../mm-image-codec/qexif

server_test_SOURCES = src/server_test.cpp

server_test_LDADD = -lpthread -lturbojpeg libcamcli.la ../libcamera/libcamera.la

