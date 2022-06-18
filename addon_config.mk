# All variables and this file are optional, if they are not present the PG and the
# makefiles will try to parse the correct values from the file system.
#
# Variables that specify exclusions can use % as a wildcard to specify that anything in
# that position will match. A partial path can also be specified to, for example, exclude
# a whole folder from the parsed paths from the file system
#
# Variables can be specified using = or +=
# = will clear the contents of that variable both specified from the file or the ones parsed
# from the file system
# += will add the values to the previous ones in the file or the ones parsed from the file 
# system
# 
# The PG can be used to detect errors in this file, just create a new project with this addon 
# and the PG will write to the console the kind of error and in which line it is

meta:
	ADDON_NAME = ofxLidarSlam
	ADDON_DESCRIPTION = Use SLAM with a Lidar
	ADDON_AUTHOR = @roymacdonald
	ADDON_TAGS = "addon" "slam" "lidar" "lidar-slam" "ouster"
	ADDON_URL = http://github.com/roymacdonald/ofxLidarSlam

common:
	# dependencies with other addons, a list of them separated by spaces 
	# or use += in several lines
	ADDON_DEPENDENCIES = ofxDropdown ofxCeresLib ofxOusterLidar
	
	# include search paths, this will be usually parsed from the file system
	# but if the addon or addon libraries need special search paths they can be
	# specified here separated by spaces or one per line using +=
	# ADDON_INCLUDES =
	
	# any special flag that should be passed to the compiler when using this
	# addon
	# ADDON_CFLAGS =
	
	# any special flag that should be passed to the compiler for c++ files when 
	# using this addon
	# ADDON_CPPFLAGS =
	
	# any special flag that should be passed to the linker when using this
	# addon, also used for system libraries with -lname
	# ADDON_LDFLAGS =
	
	# source files, these will be usually parsed from the file system looking
	# in the src folders in libs and the root of the addon. if your addon needs
	# to include files in different places or a different set of files per platform
	# they can be specified here
	# ADDON_SOURCES =
	
	# source files that will be included as C files explicitly
	# ADDON_C_SOURCES = 
	
	# source files that will be included as header files explicitly
	# ADDON_HEADER_SOURCES = 
	
	# source files that will be included as c++ files explicitly
	# ADDON_CPP_SOURCES = 
	
	# source files that will be included as objective c files explicitly
	# ADDON_OBJC_SOURCES = 
	
	# derines that will be passed to the compiler when including this addon
	
	# some addons need resources to be copied to the bin/data folder of the project
	# specify here any files that need to be copied, you can use wildcards like * and ?
	# ADDON_DATA = 
	
	# when parsing the file system looking for libraries exclude this for all or
	# a specific platform
	# ADDON_LIBS_EXCLUDE =
	
	# binary libraries, these will be usually parsed from the file system but some 
	# libraries need to passed to the linker in a specific order/
	# 
	# For example in the ofxOpenCV addon we do something like this:
	#
	# ADDON_LIBS =
	# ADDON_LIBS += libs/opencv/lib/linuxarmv6l/libopencv_legacy.a
	# ADDON_LIBS += libs/opencv/lib/linuxarmv6l/libopencv_calib3d.a
	# ...
	
	
linux64:
	# linux only, any library that should be included in the project using
	# pkg-config
	# ADDON_PKG_CONFIG_LIBRARIES =
vs:
	# After compiling copy the following dynamic libraries to the executable directory
	# only windows visual studio
	# ADDON_DLLS_TO_COPY = 
	
linuxarmv6l:
linuxarmv7l:
android/armeabi:	
android/armeabi-v7a:	
osx:
	# osx/iOS only, any framework that should be included in the project
	# ADDON_FRAMEWORKS =
	
# I am not sure if actually all these libraries need to be linked or just some, but I am too lazy right not to figure it out.
# if you find out please let me know
	ADDON_LIBS  = /usr/local/lib/libg2o_cli.dylib
	ADDON_LIBS += /usr/local/lib/libg2o_core.dylib
	ADDON_LIBS += /usr/local/lib/libg2o_hierarchical.dylib
	ADDON_LIBS += /usr/local/lib/libg2o_simulator.dylib
	ADDON_LIBS += /usr/local/lib/libg2o_solver_dense.dylib
	ADDON_LIBS += /usr/local/lib/libg2o_solver_eigen.dylib
	ADDON_LIBS += /usr/local/lib/libg2o_solver_pcg.dylib
	ADDON_LIBS += /usr/local/lib/libg2o_solver_structure_only.dylib
	ADDON_LIBS += /usr/local/lib/libg2o_stuff.dylib
	ADDON_LIBS += /usr/local/lib/libg2o_types_data.dylib
	ADDON_LIBS += /usr/local/lib/libg2o_types_icp.dylib
	ADDON_LIBS += /usr/local/lib/libg2o_types_sba.dylib
	ADDON_LIBS += /usr/local/lib/libg2o_types_sclam2d.dylib
	ADDON_LIBS += /usr/local/lib/libg2o_types_sim3.dylib
	ADDON_LIBS += /usr/local/lib/libg2o_types_slam2d.dylib
	ADDON_LIBS += /usr/local/lib/libg2o_types_slam2d_addons.dylib
	ADDON_LIBS += /usr/local/lib/libg2o_types_slam3d.dylib
	ADDON_LIBS += /usr/local/lib/libg2o_types_slam3d_addons.dylib
 	ADDON_LIBS += /usr/local/lib/libboost_filesystem.dylib



ios:
tvos:

