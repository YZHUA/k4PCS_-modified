#-------------------------------------------------
#
# Project created by QtCreator 2016-09-07T19:34:18
#
#-------------------------------------------------

QT       += core

QT       -= gui

TARGET = k4pcs
CONFIG   += console
CONFIG   -= app_bundle

TEMPLATE = app


SOURCES += \
    src/kfpcs_ia_sample_program.cpp

HEADERS += \
    include/sift_keypoint_large.h \
    include/short_filter.h \
    include/kfpcs_ia_data.h \
    include/impl/sift_keypoint_large.hpp \
    include/impl/short_filter.hpp

INCLUDEPATH += /home/yzh/pcl/pcl_180/include/pcl-1.8/pcl\

INCLUDEPATH += /home/yzh/pcl/pcl_180/include/pcl-1.8\

INCLUDEPATH += /usr/include/boost\

INCLUDEPATH += /usr/include/eigen3\

INCLUDEPATH += /usr/include/flann\

INCLUDEPATH += /usr/include/qhull\

INCLUDEPATH +=/usr/include/vtk-5.8\

CONFIG(debug,debug|release){
LIBS+=/usr/lib/x86_64-linux-gnu/libboost_system.a\
/usr/lib/x86_64-linux-gnu/libboost_filesystem.a\
/usr/lib/x86_64-linux-gnu/libboost_thread.a\
/usr/lib/x86_64-linux-gnu/libboost_date_time.a\
/usr/lib/x86_64-linux-gnu/libboost_iostreams.a\
/usr/lib/x86_64-linux-gnu/libboost_serialization.a\
/usr/lib/x86_64-linux-gnu/libboost_chrono.a\
/home/yzh/pcl/pcl_180/lib/libpcl_common.so\
/home/yzh/pcl/pcl_180/lib/libpcl_octree.so\
/home/yzh/pcl/pcl_180/lib/libpcl_io.so\
/home/yzh/pcl/pcl_180/lib/libpcl_kdtree.so\
/home/yzh/pcl/pcl_180/lib/libpcl_search.so\
/home/yzh/pcl/pcl_180/lib/libpcl_sample_consensus.so\
/home/yzh/pcl/pcl_180/lib/libpcl_filters.so\
/home/yzh/pcl/pcl_180/lib/libpcl_features.so\
/home/yzh/pcl/pcl_180/lib/libpcl_segmentation.so\
/home/yzh/pcl/pcl_180/lib/libpcl_visualization.so\
/home/yzh/pcl/pcl_180/lib/libpcl_surface.so\
/home/yzh/pcl/pcl_180/lib/libpcl_registration.so\
/home/yzh/pcl/pcl_180/lib/libpcl_keypoints.so\
/home/yzh/pcl/pcl_180/lib/libpcl_tracking.so\
/home/yzh/pcl/pcl_180/lib/libpcl_recognition.so\
/home/yzh/pcl/pcl_180/lib/libpcl_apps.so\
/home/yzh/pcl/pcl_180/lib/libpcl_outofcore.so\
/home/yzh/pcl/pcl_180/lib/libpcl_people.so\
/usr/lib/x86_64-linux-gnu/libflann_cpp.so\
/usr/lib/x86_64-linux-gnu/libqhull.so\
/usr/lib/libOpenNI.so\
/usr/lib/libvtkGenericFiltering.so\
/usr/lib/libvtkGeovis.so\
/usr/lib/libvtkCharts.so\
/usr/lib/libvtkViews.so\
/usr/lib/libvtkInfovis.so\
/usr/lib/libvtkWidgets.so\
/usr/lib/libvtkVolumeRendering.so\
/usr/lib/libvtkHybrid.so\
/usr/lib/libvtkRendering.so\
/usr/lib/libvtkImaging.so\
/usr/lib/libvtkGraphics.so\
/usr/lib/libvtkIO.so\
/usr/lib/libvtkFiltering.so\
/usr/lib/libvtkCommon.so\
/usr/lib/libvtksys.so\
/usr/lib/libQVTK.so\
/usr/lib/qt4/plugins/designer/libQVTKWidgetPlugin.so\
}
CONFIG(release,debug|release){
LIBS+=/usr/lib/x86_64-linux-gnu/libboost_system.a\
/usr/lib/x86_64-linux-gnu/libboost_filesystem.a\
/usr/lib/x86_64-linux-gnu/libboost_thread.a\
/usr/lib/x86_64-linux-gnu/libboost_date_time.a\
/usr/lib/x86_64-linux-gnu/libboost_iostreams.a\
/usr/lib/x86_64-linux-gnu/libboost_serialization.a\
/usr/lib/x86_64-linux-gnu/libboost_chrono.a\
/usr/lib/libpcl_common.so\
/usr/lib/libpcl_octree.so\
/usr/lib/libpcl_io.so\
/usr/lib/libpcl_kdtree.so\
/usr/lib/libpcl_search.so\
/usr/lib/libpcl_sample_consensus.so\
/usr/lib/libpcl_filters.so\
/usr/lib/libpcl_features.so\
/usr/lib/libpcl_segmentation.so\
/usr/lib/libpcl_visualization.so\
/usr/lib/libpcl_surface.so\
/usr/lib/libpcl_registration.so\
/usr/lib/libpcl_keypoints.so\
/usr/lib/libpcl_tracking.so\
/usr/lib/libpcl_recognition.so\
/usr/lib/libpcl_apps.so\
/usr/lib/libpcl_outofcore.so\
/usr/lib/libpcl_people.so\
/usr/lib/x86_64-linux-gnu/libflann_cpp.so\
/usr/lib/x86_64-linux-gnu/libqhull.so\
/usr/lib/libOpenNI.so\
/usr/lib/libvtkGenericFiltering.so\
/usr/lib/libvtkGeovis.so\
/usr/lib/libvtkCharts.so\
/usr/lib/libvtkViews.so\
/usr/lib/libvtkInfovis.so\
/usr/lib/libvtkWidgets.so\
/usr/lib/libvtkVolumeRendering.so\
/usr/lib/libvtkHybrid.so\
/usr/lib/libvtkRendering.so\
/usr/lib/libvtkImaging.so\
/usr/lib/libvtkGraphics.so\
/usr/lib/libvtkIO.so\
/usr/lib/libvtkFiltering.so\
/usr/lib/libvtkCommon.so\
/usr/lib/libvtksys.so\
/usr/lib/libQVTK.so\
/usr/lib/qt4/plugins/designer/libQVTKWidgetPlugin.so\
}
