QT       += core gui network

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++11

# The following define makes your compiler emit warnings if you use
# any Qt feature that has been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    main.cpp \
    widget.cpp

HEADERS += \
    widget.h

FORMS += \
    widget.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
#导入opencv 的库
INCLUDEPATH += /usr/local/include/opencv5/
INCLUDEPATH += /usr/local/lib/
LIBS += /usr/local/lib/libopencv_videoio.so \
    -lopencv_calib \
    -lopencv_core \
    -lopencv_features2d \
    -lopencv_flann \
    -lopencv_highgui \
    -lopencv_imgproc \
    -lopencv_ml \
    -lopencv_objdetect \
    -lopencv_photo \
    -lopencv_stitching \
    -lopencv_video
LIBS +=/usr/local/lib/libopencv_core.so
LIBS +=/usr/local/lib/libopencv_imgcodecs.so
