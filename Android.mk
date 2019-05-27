LOCAL_PATH := $(call my-dir)


include $(CLEAR_VARS)
LOCAL_MODULE := myopencv
LOCAL_SRC_FILES := $(LOCAL_PATH)/hdaLPR/cpp/opencv342/sdk/native/libs/armeabi-v7a/libopencv_java3.so
include $(PREBUILT_SHARED_LIBRARY)

#include $(CLEAR_VARS)

# OpenCV
OPENCV_CAMERA_MODULES:=on
OPENCV_INSTALL_MODULES:=on
include hdaLPR/cpp/opencv342/sdk/native/jni/OpenCV.mk

FILE_LIST := $(wildcard $(LOCAL_PATH)/hdaLPR/cpp/src/*.cpp)
LOCAL_SRC_FILES := $(LOCAL_PATH)/hdaLPR/cpp/javaWarpper.cpp
LOCAL_SRC_FILES += $(FILE_LIST:$(LOCAL_PATH)/%=%)

LOCAL_C_INCLUDES += $(LOCAL_PATH)/
LOCAL_C_INCLUDES += $(LOCAL_PATH)/hdaLPR/include

#LOCAL_SHARED_LIBRARIES := myopencv
LOCAL_LDLIBS += -llog

LOCAL_MODULE     := hdalpr

include $(BUILD_SHARED_LIBRARY)  

