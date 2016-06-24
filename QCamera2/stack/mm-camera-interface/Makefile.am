ACLOCAL_AMFLAGS = -I m4

lib_LTLIBRARIES = libmmcamera_interface.la

libmmcamera_interface_la_CFLAGS = -DAMSS_VERSION=$(AMSS_VERSION)
libmmcamera_interface_la_CFLAGS += -DMSM_CAMERA_GCC
libmmcamera_interface_la_CFLAGS += -fPIC
libmmcamera_interface_la_CFLAGS += -DUNIX_PATH_MAX=108
libmmcamera_interface_la_CFLAGS += -DALOGD=fprintf
libmmcamera_interface_la_CFLAGS += -Dstrlcpy=g_strlcpy

libmmcamera_interface_la_CFLAGS += -I ${WORKSPACE}/hardware/qcom/camera/QCamera2/stack/mm-camera-interface/inc
libmmcamera_interface_la_CFLAGS += -I ${WORKSPACE}/hardware/qcom/camera/QCamera2/stack/common
libmmcamera_interface_la_CFLAGS += -I ${WORKSPACE}/system/core/include
libmmcamera_interface_la_CFLAGS += -I ${WORKSPACE}/framework/av/include
libmmcamera_interface_la_CFLAGS += -I ${WORKSPACE}/hardware/libhardware/include


libmmcamera_interface_la_SOURCES = src/mm_camera_interface.c
libmmcamera_interface_la_SOURCES += src/mm_camera.c
libmmcamera_interface_la_SOURCES += src/mm_camera_channel.c
libmmcamera_interface_la_SOURCES += src/mm_camera_stream.c
libmmcamera_interface_la_SOURCES += src/mm_camera_thread.c
libmmcamera_interface_la_SOURCES += src/mm_camera_sock.c
libmmcamera_interface_la_SOURCES += src/cam_intf.c

libmmcamera_interface_la_LDFLAGS = -shared -ldl -lglib-2.0

