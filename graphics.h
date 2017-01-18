// Graphics for cart-pusher, lifted from my project EG, lifted from my project TFPS.

#ifndef _EG_GRAPHICS_H
#define _EG_GRAPHICS_H

#include "physics.h"
#include "compatibility.h"

extern "C" EXPORT void begin_frame();
extern "C" EXPORT void begin_overlay();
extern "C" EXPORT void end_frame();
extern "C" EXPORT float get_text_width(char* text, float height);
extern "C" EXPORT void draw_with_font(char* text, float x, float y, float height);
extern "C" EXPORT void set_color(float r, float g, float b, float a);
extern "C" EXPORT void translate(float x, float y, float z);
extern "C" EXPORT void draw_triangle(float* coords, int texture);
extern "C" EXPORT void draw_sphere(float x, float y, float z, float radius);
extern "C" EXPORT void draw_triangles(int count, float* coords, float* texture_coords, int texture_index);
extern "C" EXPORT void draw_box(float* coords, int texture_index);
extern "C" EXPORT void warp_mouse(int x, int y);
extern "C" EXPORT int get_event();
extern "C" EXPORT int graphics_init(int fullscreen);
extern "C" EXPORT int graphics_close();
extern "C" EXPORT void play_music(char*);
extern "C" EXPORT void play_sound(char*);

extern "C" EXPORT void gettimeofday_wrapper(long long* result);

#endif

