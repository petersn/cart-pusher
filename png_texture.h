// This one file (png_texture.cpp) is free and unencumbered software
// released into the public domain.
// I wrote this trivial little header file to integrate with the code.

#include <GL/gl.h>
#include <png.h>

extern "C" GLuint png_texture_load(const char * file_name, int * width_height, int use_alpha);

