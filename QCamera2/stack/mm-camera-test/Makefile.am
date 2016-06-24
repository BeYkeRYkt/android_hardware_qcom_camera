ACLOCAL_AMFLAGS = -I m4
# mm-qcamera-app
bin_PROGRAMS = mm-qcamera-app

mm_qcamera_app_CFLAGS = $(DEBUG_CFLAGS)
mm_qcamera_app_CFLAGS += -DAMSS_VERSION=$(AMSS_VERSION)
mm_qcamera_app_CFLAGS += -DUSE_ION
mm_qcamera_app_CFLAGS += -DCAMERA_ION_HEAP_ID=ION_IOMMU_HEAP_ID
mm_qcamera_app_CFLAGS += -DCAMERA_GRALLOC_HEAP_ID=GRALLOC_USAGE_PRIVATE_MM_HEAP
mm_qcamera_app_CFLAGS += -DCAMERA_GRALLOC_FALLBACK_HEAP_ID=GRALLOC_USAGE_PRIVATE_IOMMU_HEAP
mm_qcamera_app_CFLAGS += -DCAMERA_ION_FALLBACK_HEAP_ID=ION_IOMMU_HEAP_ID
mm_qcamera_app_CFLAGS += -DCAMERA_GRALLOC_CACHING_ID=0
mm_qcamera_app_CFLAGS += -DNUM_RECORDING_BUFFERS=9
mm_qcamera_app_CFLAGS += -DALOGD=fprintf
#mm_qcamera_app_CFLAGS += -DPOINTER_OF_PARAM=POINTER_OF_META

mm_qcamera_app_CPPFLAGS = -I ${WORKSPACE}/hardware/qcom/camera/QCamera2/stack/mm-camera-test/inc
mm_qcamera_app_CPPFLAGS += -I ${WORKSPACE}/hardware/qcom/camera/QCamera2/stack/mm-camera-interface/inc
mm_qcamera_app_CPPFLAGS += -I ${WORKSPACE}/hardware/qcom/camera/QCamera2/stack/common
mm_qcamera_app_CPPFLAGS += -I ${WORKSPACE}/hardware/qcom/camera/mm-image-codec/qomx_core
mm_qcamera_app_CPPFLAGS += -I ${WORKSPACE}/frameworks/native/include/media/openmax
mm_qcamera_app_CPPFLAGS += -I ${WORKSPACE}/hardware/qcom/camera/mm-image-codec/qexif
mm_qcamera_app_CPPFLAGS += -I ${WORKSPACE}/system/core/include
mm_qcamera_app_CPPFLAGS += -I ${WORKSPACE}/framework/av/include
mm_qcamera_app_CPPFLAGS += -I ${WORKSPACE}/hardware/libhardware/include

#sources
mm_qcamera_app_SOURCES = src/mm_qcamera_main_menu.c
mm_qcamera_app_SOURCES += src/mm_qcamera_app.c
mm_qcamera_app_SOURCES += src/mm_qcamera_unit_test.c
mm_qcamera_app_SOURCES += src/mm_qcamera_video.c
mm_qcamera_app_SOURCES += src/mm_qcamera_preview.c
mm_qcamera_app_SOURCES += src/mm_qcamera_snapshot.c
mm_qcamera_app_SOURCES += src/mm_qcamera_rdi.c
mm_qcamera_app_SOURCES += src/mm_qcamera_reprocess.c
mm_qcamera_app_SOURCES += src/mm_qcamera_queue.c
mm_qcamera_app_SOURCES += src/mm_qcamera_socket.c
mm_qcamera_app_SOURCES += src/mm_qcamera_commands.c

mm_qcamera_app_LDFLAGS = -ldl -lpthread -lglib-2.0

mm_qcamera_app_LDADD =  ../mm-camera-interface/libmmcamera_interface.la

