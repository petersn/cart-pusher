
CPPFLAGS=-g `sdl-config --cflags` -fPIC -std=c++0x -O3 -ffast-math -pthread -I/usr/include/bullet -I/usr/local/include/bullet
LIBS=`sdl-config --libs` -lGL -lGLU -lpng -lSDL -L/usr/local/lib -lBulletDynamics -lBulletCollision -lLinearMath -lSDL_mixer
CXX=g++

all: libcartpusher.so

libcartpusher.so: graphics.o physics.o png_texture.o Makefile
	$(CXX) -shared -Wl,-soname,$@ -o $@ graphics.o physics.o png_texture.o $(LIBS)

.PHONY: clean
clean:
	rm -f *.o libcartpusher.so

