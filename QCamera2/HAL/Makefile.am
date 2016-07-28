ACLOCAL_AMFLAGS = -I m4

# build camera-hal Library
lib_LTLIBRARIES = libqcamera2.la

libqcamera2_la_CXXFLAGS  = -fPIC
libqcamera2_la_CXXFLAGS += -g

libqcamera2_la_CXXFLAGS += -I ../stack/common
libqcamera2_la_CXXFLAGS += -I ../stack/mm-camera-interface/inc
libqcamera2_la_CXXFLAGS += -I ../../mm-image-codec/qexif
libqcamera2_la_CXXFLAGS += -I ../../mm-image-codec/qomx_core
libqcamera2_la_CXXFLAGS += -I ../util
libqcamera2_la_CXXFLAGS += -I ../..

libqcamera2_la_CXXFLAGS += -DLINUX_ENABLED
libqcamera2_la_CXXFLAGS += -DHAVE_PTHREADS
libqcamera2_la_CXXFLAGS += -DANDROID_SMP=0
libqcamera2_la_CXXFLAGS += -I$(WORKSPACE)/system/core/include

libqcamera2_la_SOURCES  = QCamera2Factory.cpp
libqcamera2_la_SOURCES += QCamera2Hal.cpp
libqcamera2_la_SOURCES += QCamera2HWI.cpp
libqcamera2_la_SOURCES += QCameraMem.cpp
libqcamera2_la_SOURCES += ../util/QCameraQueue.cpp
libqcamera2_la_SOURCES += ../util/QCameraCmdThread.cpp
libqcamera2_la_SOURCES += QCameraStateMachine.cpp
libqcamera2_la_SOURCES += QCameraChannel.cpp
libqcamera2_la_SOURCES += QCameraStream.cpp
libqcamera2_la_SOURCES += QCameraPostProc.cpp
libqcamera2_la_SOURCES += QCamera2HWICallbacks.cpp
libqcamera2_la_SOURCES += QCameraParameters.cpp
libqcamera2_la_SOURCES += QCameraThermalAdapter.cpp

libqcamera2_la_LDFLAGS = -shared

libqcamera2_la_LIBADD  = -ldl -lpthread -lglib-2.0 -lutils -lcutils -lcamparams -lhardware

libqcamera2_la_LIBADD += ../stack/mm-camera-interface/libmmcamera_interface.la
libqcamera2_la_LIBADD += ../stack/mm-jpeg-interface/libmmjpeg_interface.la
