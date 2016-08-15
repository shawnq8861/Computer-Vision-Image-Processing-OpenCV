TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
    alignEyeCamera.cpp

unix:!macx: LIBS += -L$$PWD/../../../../../usr/lib/openCV31Build/lib/ -lopencv_core
unix:!macx: LIBS += -L$$PWD/../../../../../usr/lib/openCV31Build/lib/ -lopencv_highgui
unix:!macx: LIBS += -L$$PWD/../../../../../usr/lib/openCV31Build/lib/ -lopencv_imgproc

INCLUDEPATH += $$PWD/../../../../../usr/lib/openCV31Build/include
DEPENDPATH += $$PWD/../../../../../usr/lib/openCV31Build/include

unix:!macx: LIBS += -L$$PWD/../../../../../usr/lib/ -lueye_api

INCLUDEPATH += $$PWD/../../../../../usr/include
DEPENDPATH += $$PWD/../../../../../usr/include
