
QMAKE_CXXFLAGS += -std=c++11
INCLUDEPATH += /opt/ros/kinetic/include /home/fshs/catkin_ws/devel/include
DEPENDPATH += /opt/ros/kinetic/include
LIBS += -L/opt/ros/kinetic/lib -lroscpp -lroslib -lrosconsole -lroscpp_serialization -lrostime
LIBS += /opt/ros/kinetic/lib/libxmlrpcpp.so \
        /opt/ros/kinetic/lib/libcpp_common.so \
        /opt/ros/kinetic/lib/librosconsole_log4cxx.so
        /opt/ros/kinetic/lib/librosconsole_backend_interface.so\
