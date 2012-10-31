CC	= g++
CFLAGS = -I. -I/sw/include
# Linux
LDFLAGS =  -L/sw/lib  -lm

#MacOS
#LDFLAGS = -L/usr/X11R6/lib -L/sw/lib -lGL -lGLU -lglut -lm -framework Cocoa -framework OpenGL -bind_at_load -lpng

OBJS = tracker_test.o tracker.o timer.o 

all: tacker_test comport_test

clean: 
	rm -f tracker_test *.o *~

tacker_test: ${OBJS}
	${CC} ${OBJS} -o tracker_test ${LDFLAGS}

comport_test: comport.o comport_test.o
	${CC} comport.o comport_test.o -o comport_test

%.o: %.cpp %.hpp
	${CC} -c $< -o $@ ${CFLAGS}


