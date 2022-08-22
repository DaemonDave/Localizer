# ATLAS LOCATIONS 
# For CBLAS and CLAPACK Fcns
ATLASLIB = ../ATLAS/lib/Linux_P4SSE2_2
ATLASINC = ../ATLAS/include

lib_LIBRARIES = liblocalizer.a

liblocalizer_a_SOURCES =  filter.c  \
                           kin_model.c \
			   LatLong-UTMconversion.c  \
			   localize.c \
			   matrix.c  \
			   sensor.c \
			   sensor_gps.c \
			   sensor_imu.c \
			   sensor_odom.c \
			   sincos.c \
			   state_vector.c \
			   transducer.c \
			   _mathprnt.c \
			   _posemath.c 
				
										
# set the include path found by configure
INCLUDES= $(all_includes) 
# 
LDADD = -lm  
liblocalizer_a_LIBADD =  
#  


# the library search path.
liblocalizer_a_LDFLAGS = $(all_libraries) 

AM_CFLAGS = -Wall -g -fpic -I$(ATLASINC)
AM_LDFLAGS = 
