#-------------------------------------------------
#
# Project created by QtCreator 2015-06-09T20:59:54
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets


TARGET = ytracking
TEMPLATE = app


INCLUDEPATH += "../opencv/build/include/opencv2"
debug {
    LIBS += -L"../opencv/build/x64/vc12/lib" -lopencv_core2410d
}

release {
    LIBS += -L"../opencv/build/x64/vc12/lib" -lopencv_core2410
}


SOURCES += main.cpp\
        mainwindow.cpp

HEADERS  += mainwindow.h

FORMS    += mainwindow.ui






