# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "mavros_msgs;roscpp;std_msgs;sensor_msgs;message_runtime".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lquadcopter_control".split(';') if "-lquadcopter_control" != "" else []
PROJECT_NAME = "quadcopter_control"
PROJECT_SPACE_DIR = "/usr/local"
PROJECT_VERSION = "0.0.0"
