

QT += core network

TEMPLATE = app

INCLUDEPATH += "../opencv/build/include" \
    "pthread_prebuild/include"

LIBS += -L"../opencv/build/x64/vc12/lib" \
    -lopencv_imgproc2410d \
    -lopencv_highgui2410d \
    -lopencv_video2410d \
    -lopencv_core2410d

LIBS += -L"../ytracking/pthread_prebuild/lib/x64" \
    -lpthreadVC2



HEADERS += \
    tracker/Ctracker.h \
    tracker/HungarianAlg.h \
    tracker/Kalman.h \
    tracker/ytracker.h \
    opencvblobslib/library/blob.h \
    opencvblobslib/library/BlobContour.h \
    opencvblobslib/library/BlobLibraryConfiguration.h \
    opencvblobslib/library/BlobOperators.h \
    opencvblobslib/library/BlobResult.h \
    opencvblobslib/library/ComponentLabeling.h \
    package_bgs/PBAS/PBAS.h \
    package_bgs/PBAS/PixelBasedAdaptiveSegmenter.h \
    package_bgs/IBGS.h \
    tracker/streamer.h


SOURCES += \
    tracker/Ctracker.cpp \
    tracker/HungarianAlg.cpp \
    tracker/Kalman.cpp \
    tracker/ytracker.cpp \
    opencvblobslib/library/blob.cpp \
    opencvblobslib/library/BlobContour.cpp \
    opencvblobslib/library/BlobOperators.cpp \
    opencvblobslib/library/BlobResult.cpp \
    opencvblobslib/library/ComponentLabeling.cpp \
    package_bgs/PBAS/PBAS.cpp \
    package_bgs/PBAS/PixelBasedAdaptiveSegmenter.cpp \
    tracker/streamer.cpp \
    main.cpp



