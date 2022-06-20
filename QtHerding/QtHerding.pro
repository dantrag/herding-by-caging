#-------------------------------------------------
#
# Project created by QtCreator 2016-11-26T19:58:45
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = QtHerding
TEMPLATE = app

INCLUDEPATH += include
#/usr/include/eigen3
QMAKE_CXXFLAGS += -std=c++11 -I/usr/local/include -I/usr/local/include/ompl-1.5 -I/usr/include/eigen3
#-I/usr/include -I/usr/include/python2.7
QMAKE_CXX = g++
QMAKE_LINK = g++ -std=c++11
#QMAKE_LFLAGS += -lboost_system -lboost_thread -L/usr/local/lib/x86_64-linux-gnu -lompl -lompl_app -lompl_app_base -Wl,-rpath /usr/local/lib/x86_64-linux-gnu

LIBS += -L/usr/local/lib -Wl,-rpath /usr/local/lib -lboost_system -lboost_thread -lompl -lompl_app -lompl_app_base -lCGAL -lgmp -lmpfr
#LIBS += -L/usr/local/lib/x86_64-linux-gnu -Wl,-rpath /usr/local/lib/x86_64-linux-gnu -lboost_system -lboost_thread -lCGAL -lgmp -lmpfr

SOURCES += main.cpp\
        mainwindow.cpp \
    backend/HerdingSampler.cpp \
    backend/HerdingPlanning.cpp \
    backend/ashapes/ashapes2d.cpp \
    polygon.cpp \
    include/utilities/munkres/munkres.cpp \
    planners/RRTNoInterpolation.cpp \
    planners/RRTInterpolation.cpp \
    planners/RRTRandomTranslate.cpp \
    ui/InputWidget.cpp \
    ui/singleMode/NormalModeWidget.cpp \
    ui/singleMode/NormalModeStartWidget.cpp \
    ui/singleMode/NormalModeGoalWidget.cpp \
    ui/statMode/StatisticsModeGoalWidget.cpp \
    ui/statMode/StatisticsModeStartWidget.cpp \
    ui/statMode/StatisticsModeWidget.cpp \
    ui/statMode/StatisticsModeRandomStartWidget.cpp \
    ui/statMode/StatisticsModeCustomStartWidget.cpp

HEADERS  += mainwindow.h \
    backend/HerdingSampler.h \
    backend/HerdingPlanning.h \
#    backend/ashapes/ashapes2d.hpp \
    backend/ashapes/ashapes2d.h \
    polygon.h \
    include/dionysus.h \
    include/geometry/distances.h \
    include/geometry/distances.hpp \
    include/geometry/euclidean.h \
    include/geometry/euclidean.hpp \
    include/geometry/kinetic-sort.h \
    include/geometry/kinetic-sort.hpp \
    include/geometry/l2distance.h \
    include/geometry/linalg.h \
    include/geometry/linalg.hpp \
    include/geometry/linear-kernel.h \
    include/geometry/number-traits.h \
    include/geometry/polynomial.h \
    include/geometry/polynomial.hpp \
    include/geometry/rational-function.h \
    include/geometry/rational-function.hpp \
    include/geometry/simulator.h \
    include/geometry/simulator.hpp \
    include/geometry/weighted-cechdistance.h \
    include/geometry/weighted-l2distance.h \
    include/topology/chain.h \
    include/topology/chain.hpp \
    include/topology/cohomology-persistence.h \
    include/topology/cohomology-persistence.hpp \
    include/topology/complex-traits.h \
    include/topology/conesimplex.h \
    include/topology/conesimplex.hpp \
    include/topology/cycles.h \
    include/topology/dynamic-persistence.h \
    include/topology/dynamic-persistence.hpp \
    include/topology/field-arithmetic.h \
    include/topology/filtration.h \
    include/topology/filtration.hpp \
    include/topology/image-zigzag-persistence.h \
    include/topology/image-zigzag-persistence.hpp \
    include/topology/lowerstarfiltration.h \
    include/topology/lsvineyard.h \
    include/topology/lsvineyard.hpp \
    include/topology/order.h \
    include/topology/persistence-diagram.h \
    include/topology/persistence-diagram.hpp \
    include/topology/rips.h \
    include/topology/rips.hpp \
    include/topology/simplex.h \
    include/topology/simplex.hpp \
    include/topology/static-persistence.h \
    include/topology/static-persistence.hpp \
    include/topology/static-persistence.hpp~ \
    include/topology/vineyard.h \
    include/topology/vineyard.hpp \
    include/topology/weighted-rips.h \
    include/topology/zigzag-persistence.h \
    include/topology/zigzag-persistence.hpp \
    include/utilities/munkres/matrix.h \
    include/utilities/munkres/matrix.hpp \
    include/utilities/munkres/munkres.h \
    include/utilities/binaryheap.h \
    include/utilities/circular_list.h \
    include/utilities/consistencylist.h \
    include/utilities/consistencylist.hpp \
    include/utilities/containers.h \
    include/utilities/counter.h \
    include/utilities/counter.hpp \
    include/utilities/eventqueue.h \
    include/utilities/indirect.h \
    include/utilities/log.h \
    include/utilities/memory.h \
    include/utilities/orderlist.h \
    include/utilities/orderlist.hpp \
    include/utilities/property-maps.h \
    include/utilities/timer.h \
    include/utilities/types.h \
#    ../../ompl-1.2.1-Source/tests/resources/config.h \
    planners/RRTNoInterpolation.h \
    planners/RRTInterpolation.h \
    planners/RRTRandomTranslate.h \
    ui/InputWidget.h \
    ui/singleMode/NormalModeWidget.h \
    ui/singleMode/NormalModeStartWidget.h \
    ui/singleMode/NormalModeGoalWidget.h \
    ui/statMode/StatisticsModeGoalWidget.h \
    ui/statMode/StatisticsModeStartWidget.h \
    ui/statMode/StatisticsModeWidget.h \
    ui/statMode/StatisticsModeRandomStartWidget.h \
    ui/statMode/StatisticsModeCustomStartWidget.h

FORMS    += mainwindow.ui \
    ui/singleMode/NormalModeStartWidget.ui \
    ui/statMode/StatisticsModeStartWidget.ui \
    ui/singleMode/NormalModeGoalWidget.ui \
    ui/statMode/StatisticsModeGoalWidget.ui \
    ui/singleMode/NormalModeWidget.ui \
    ui/statMode/StatisticsModeWidget.ui \
    ui/statMode/StatisticsModeCustomStartWidget.ui \
    ui/statMode/StatisticsModeRandomStartWidget.ui

OTHER_FILES += \
    include/geometry/NOTES \
    meshes/environments/Barriers_easy_env.dae \
    meshes/environments/BoundingBox_planar_env.dae \
    meshes/environments/BugTrap_planar_env.dae \
    meshes/environments/H_planar_env.dae \
    meshes/environments/Maze_planar_env.dae \
    meshes/environments/Passage_planar_env.dae \
    meshes/environments/RandomPolygons_planar_env.dae \
    meshes/environments/UniqueSolutionMaze_env.dae \
    meshes/robots/car1_planar_robot.dae \
    meshes/environments/BugTrap_really_planar_env.dae \
    meshes/robots/car1_planar_small_robot.dae
