TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

# Need to figure this out; library is only shareable, so we say LIB_SYS_LIBS and it works
ifeq (linux-x86_64, $(findstring linux-x86_64, $(T_A)))
SHARED_LIBRARIES=NO
endif

LIB_INSTALLS_Linux += $(TUCSENSDK)/sdk/tuusb.conf
LIB_INSTALLS_Linux += $(TUCSENSDK)/sdk/libTUCam.so
LIB_INSTALLS_LINUX += $(OPENCV_LIB)/libopencv_core.so
LIB_INSTALLS_LINUX += $(OPENCV_LIB)/libopencv_highgui.so
LIB_INSTALLS_LINUX += $(OPENCV_LIB)/libopencv_imgproc.so

# LIB_INSTALLS_Linux += $(TUCSENSDK)/sdk/libTUCam.so.1
# LIB_INSTALLS_Linux += $(TUCSENSDK)/sdk/libTUCam.so.1.0
# LIB_INSTALLS_Linux += $(TUCSENSDK)/sdk/libTUCam.so.1.0.0

LIBRARY_IOC_Linux += tucsen
LIB_SRCS += tucsen.cpp
LIB_SYS_LIBS_Linux += TUCam

DBD += tucsenSupport.dbd

# Add locally compiled object code
PROD_LIBS += tucsen
PROD_SYS_LIBS_Linux += TUCam

include $(ADCORE)/ADApp/commonLibraryMakefile

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

