TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
    lensCharacterization.cpp

INCLUDEPATH += C:\\openCVBuild\\install\\include
INCLUDEPATH += C:\\IDS\\Develop\\include
INCLUDEPATH += C:\\GalilLib\\gclib\\include

LIBS += "C:/openCVBuild/install/x86/mingw/lib/libopencv_core310.dll.a"
LIBS += "C:/openCVBuild/install/x86/mingw/lib/libopencv_highgui310.dll.a"
LIBS += "C:/openCVBuild/install/x86/mingw/lib/libopencv_imgproc310.dll.a"
LIBS += "C:/openCVBuild/install/x86/mingw/lib/libopencv_features2d310.dll.a"
LIBS += "C:/openCVBuild/install/x86/mingw/lib/libopencv_calib3d310.dll.a"
LIBS += "C:/openCVBuild/install/x86/mingw/lib/libopencv_imgcodecs310.dll.a"
LIBS += "C:/IDS/Develop/Lib/uEye_api.lib"

win32: LIBS += -L$$PWD/../../GalilLib/gclib/lib/dynamic/x86/ -lgclib

INCLUDEPATH += $$PWD/../../GalilLib/gclib/lib/dynamic/x86
DEPENDPATH += $$PWD/../../GalilLib/gclib/lib/dynamic/x86

win32: LIBS += -L$$PWD/../../GalilLib/gclib/lib/dynamic/x86/ -lgclibo

INCLUDEPATH += $$PWD/../../GalilLib/gclib/lib/dynamic/x86
DEPENDPATH += $$PWD/../../GalilLib/gclib/lib/dynamic/x86
