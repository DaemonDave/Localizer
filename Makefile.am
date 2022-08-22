# not a GNU package. You can remove this line, if
# have all needed files, that a GNU package needs
AUTOMAKE_OPTIONS = foreign 1.4


SUBDIRS = src  

##
# Include for local install of headers and libraries for inter development testing.
##
include $(top_srcdir)/install-local.mk
AM_CPPFLAGS  = -I$(INC_DIR)
AM_CFLAGS 	= -I$(INC_DIR)
AM_LDFLAGS  = -L$(LIB_DIR)
