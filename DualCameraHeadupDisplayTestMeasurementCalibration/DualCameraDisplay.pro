TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
    dualCameraDisplay.cpp

unix:!macx: LIBS += -L$$PWD/../../../../../usr/lib/openCV31Build/lib/ -lopencv_core
unix:!macx: LIBS += -L$$PWD/../../../../../usr/lib/openCV31Build/lib/ -lopencv_highgui
unix:!macx: LIBS += -L$$PWD/../../../../../usr/lib/openCV31Build/lib/ -lopencv_imgproc
unix:!macx: LIBS += -L$$PWD/../../../../../usr/lib/openCV31Build/lib/ -lopencv_calib3d
unix:!macx: LIBS += -L$$PWD/../../../../../usr/lib/openCV31Build/lib/ -lopencv_imgcodecs

INCLUDEPATH += $$PWD/../../../../../usr/lib/openCV31Build/include
DEPENDPATH += $$PWD/../../../../../usr/lib/openCV31Build/include

unix:!macx: LIBS += -L$$PWD/../../../../../usr/lib/ -lueye_api

INCLUDEPATH += $$PWD/../../../../../usr/include
DEPENDPATH += $$PWD/../../../../../usr/include
