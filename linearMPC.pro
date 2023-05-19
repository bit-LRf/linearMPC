QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++11

# The following define makes your compiler emit warnings if you use
# any Qt feature that has been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

INCLUDEPATH += /home/lrf/QT/QT-ROS/linearMPC/linearMPC/Eigen

INCLUDEPATH += /opt/ros/melodic/include
DEPENDPATH += /opt/ros/melodic/include
LIBS += -L/opt/ros/melodic/lib -lroscpp -lroslib -lrosconsole -lroscpp_serialization -lrostime
LIBS += -L/home/lrf/tools/osqp-eigen/osqp-eigen/build/lib -lOsqpEigen
LIBS += -L/home/lrf/tools/osqp/osqp/build/out -losqp

# You can also make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    main.cpp \
    callbackFunctions.cpp \
    getDiscreteMatrix.cpp \
    getNextState.cpp \
    getParameters.cpp \
    getReference.cpp \
    LinearMPC_fun.cpp \
    mod2pi.cpp \
    mpcOptmizer_single_shooting.cpp \
    mpcOptmizer_multiple_shooting.cpp

HEADERS += \
    LinearMPC_header.h


