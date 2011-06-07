TEMPLATE	= lib
LANGUAGE	= C++
CONFIG += qt plugin 
QT += qt3support
DESTDIR = lib
MOC_DIR = build
OBJECTS_DIR = build

INCLUDEPATH += include ui
DEPENDPATH += src include include/gripper_optimization

UI_DIR = ui/gripper_optimization

HEADERS += include/gripper_optimization/optimization_viewer_plugin.h \
           include/gripper_optimization/optimization_viewer_dialog.h

SOURCES += src/optimization_viewer_plugin.cpp \
           src/optimization_viewer_dialog.cpp \
           src/gripperDesigns.cpp

FORMS += ui/optimization_viewer_dialog.ui

GRASPIT_CFLAGS = $$system(rospack export --lang=cpp --attrib=cflags graspit)
QMAKE_CXXFLAGS += $$GRASPIT_CFLAGS

