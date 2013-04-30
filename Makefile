CC = g++
ifeq ($(shell sw_vers 2>/dev/null | grep Mac | awk '{ print $$2}'),Mac)
	CFLAGS = -g -DGL_GLEXT_PROTOTYPES -I./include/ -I/usr/X11/include -DOSX
	LDFLAGS = -framework GLUT -framework OpenGL \
    	-L"/System/Library/Frameworks/OpenGL.framework/Libraries" \
    	-lm -lstdc++ -L./lib/mac -lfreeimage
else
	CFLAGS = -g -DGL_GLEXT_PROTOTYPES -Iglut-3.7.6-bin
	LDFLAGS = -L./lib/nix -lfreeimage
endif
	
RM = /bin/rm -f 
all: main 
ALL_OBJS = obj/main.o obj/particles.o obj/constraints.o obj/spring.o
main: $(ALL_OBJS)
	$(CC) $(CFLAGS) -o final $(ALL_OBJS) $(LDFLAGS) 
MAIN_DEP = include/Particles.h include/Constraints.h include/Spring.h
obj/main.o: src/main.cpp $(MAIN_DEP)
	$(CC) $(CFLAGS) -c src/main.cpp -o obj/main.o
PART_DEP = include/Spring.h
obj/particles.o: src/Particles.cpp $(PART_DEP)
	$(CC) $(CFLAGS) -c src/Particles.cpp -o obj/particles.o
CONST_DEP = include/Particles.h include/Spring.h
obj/constraints.o: src/Constraints.cpp $(CONST_DEP)
	$(CC) $(CFLAGS) -c src/Constraints.cpp -o obj/constraints.o
SPRING_DEP = include/Particles.h
obj/spring.o: src/Spring.cpp $(SPRING_DEP)
	$(CC) $(CFLAGS) -c src/Spring.cpp -o obj/spring.o
clean: 
	$(RM) *.o final




