// Compatibility for installing for Windows.

#ifdef WINDOWS_BUILD
#define EXPORT __declspec(dllexport)
#else
#define EXPORT
#endif

#ifndef WINDOWS_BUILD
#include <SDL/SDL.h>
//#include <SDL/SDL_mixer.h>
#include <GL/gl.h>
#include <GL/glu.h>
#else
//#include <SDL/SDL_mixer.h>
#include <SDL.h>
#include <GL/gl.h>
#include <GL/glu.h>
#endif

typedef double Real;

