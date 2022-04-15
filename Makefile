#
# 'make depend' uses makedepend to automatically generate dependencies
#               (dependencies are added to end of Makefile)
# 'make'        build executable file 'mycc'
# 'make clean'  removes all .o and executable files
#
CC = g++
CFLAGS = -Wall -g -std=c++11
INCLUDES = -I. -I${CONDA_PREFIX}/include -I${CONDA_PREFIX}/include/eigen3
LFLAGS = -L/${CONDA_PREFIX}/lib
LIBS = -lpdalcpp -lgdal -lcpd -lfgt

# define the C source files
SRCS = ./src/App.cpp \
	   ./src/Atlas.cpp \
	   ./src/Atlas.hpp \
	   ./src/Grid.cpp \
	   ./src/Grid.hpp \
	   ./src/SrsTransform.cpp \
	   ./src/SrsTransform.hpp \
	   ./src/Types.hpp

#
OBJS = $(SRCS:.cpp=.o)

# define the executable file
MAIN = atlas-cpd

#
# The following part of the makefile is generic; it can be used to
# build any executable just by changing the definitions above and by
# deleting dependencies appended to the file from 'make depend'
#

.PHONY: depend clean

all:    $(MAIN)

$(MAIN): $(OBJS)
	$(CC) $(CFLAGS) $(INCLUDES) -o $(MAIN) $(OBJS) $(LFLAGS) $(LIBS)

# this is a suffix replacement rule for building .o's from .c's
# it uses automatic variables $<: the name of the prerequisite of
# the rule(a .c file) and $@: the name of the target of the rule (a .o file)
# (see the gnu make manual section about automatic variables)
.cpp.o:
	$(CC) $(CFLAGS) $(INCLUDES) -c $<  -o $@

clean:
	$(RM) *.o *~ $(MAIN)

depend: $(SRCS)
	makedepend $(INCLUDES) $^

install:
	cp $(MAIN) /usr/bin
# DO NOT DELETE THIS LINE -- make depend needs it

