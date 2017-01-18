// Graphics for cart-pusher, lifted from my project EG, lifted from my project TFPS.

#include "graphics.h"
#include "png_texture.h"

using namespace std;
#include <iostream>
#include <list>
#include <math.h>
#include <sys/time.h>

#ifndef M_PI
#define M_PI           3.14159265358979323846
#endif

//#define CAMERA_SMOOTHING_FACTOR 0.75
#define CAMERA_SMOOTHING_FACTOR 0.7

GLfloat LightAmbient[]  = { 0.2f, 0.2f, 0.2f, 1.0f };
GLfloat LightDiffuse[]  = { 0.7f, 0.7f, 0.7f, 1.0f };
GLfloat LightPosition[] = { 0.0f, 0.0f, 0.0f, 1.0f };

struct Event {
	int type, key, button;
};

// Global storage.
SDL_Surface* screen;
EXPORT int screen_width, screen_height;
EXPORT int mouse_x, mouse_y;
EXPORT Event last_event;
EXPORT float camera_x = 0, camera_y = 0, camera_z = 0;
EXPORT float camera_tilt = 0, camera_facing = 0;
EXPORT float real_camera_tilt = 0, real_camera_facing = 0;

int font_map[256];
int font_width[256];
int font_height[256];

GLint viewport[4];
GLdouble modelMatrix[16];
GLdouble projMatrix[16];


extern "C" EXPORT void begin_frame() {
	if (SDL_MUSTLOCK(screen) && SDL_LockSurface(screen) < 0)
		return;

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	// Draw 3D stuff.
	real_camera_facing = (real_camera_facing * (1.0 - CAMERA_SMOOTHING_FACTOR)) + (camera_facing * CAMERA_SMOOTHING_FACTOR);
	real_camera_tilt   = (real_camera_tilt   * (1.0 - CAMERA_SMOOTHING_FACTOR)) + (camera_tilt   * CAMERA_SMOOTHING_FACTOR);

	// Render in here.
	glRotatef(-90,                1.0f, 0.0f, 0.0f);
	glRotatef(real_camera_tilt,   1.0f, 0.0f, 0.0f);
	glRotatef(real_camera_facing, 0.0f, 0.0f, 1.0f);

	glGetIntegerv(GL_VIEWPORT, viewport);
	glGetDoublev(GL_MODELVIEW_MATRIX, modelMatrix);
	glGetDoublev(GL_PROJECTION_MATRIX, projMatrix);

	glTranslatef(-camera_x, -camera_y, -camera_z);
}

extern "C" EXPORT void begin_overlay() {
	// Switch to 2D mode.
	glDisable(GL_DEPTH_TEST);
#ifdef USE_LIGHTS
	glDisable(GL_LIGHTING);
#endif
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	glOrtho(0, screen_width, screen_height, 0, 0, 1);
	glMatrixMode (GL_MODELVIEW);
	glLoadIdentity();
}

extern "C" EXPORT void end_frame() {
	// Switch back to 3D mode.
	glEnable(GL_DEPTH_TEST);
#ifdef USE_LIGHTS
	glEnable(GL_LIGHTING);
#endif
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();

	SDL_GL_SwapBuffers();
	if (SDL_MUSTLOCK(screen))
		SDL_UnlockSurface(screen);
	SDL_Flip(screen);
}

extern "C" EXPORT float get_text_width(char* text, float height) {
	float x = 0;
	while (*text) {
		if (font_map[*text] == -1) {
			text++;
			continue;
		}
		float width_to_height = font_width[*text] / (float) font_height[*text];
		float width = height * width_to_height;
		x += width;
		text++;
	}
	return x;
}

extern "C" EXPORT void draw_with_font(char* text, float x, float y, float height) {
	glEnable(GL_TEXTURE_2D);
//	glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
	glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
	float original_x = x;
	while (*text) {
		// If we get a newline, then process it appropriately.
		if (*text == 10) {
			x = original_x; // Carriage return.
			y += height; // Line feed.
			text++;
			continue;
		}
		if (font_map[*text] == -1) {
			text++;
			continue;
		}
		float rescale = 1.0 / font_height[*text];
		float width_to_height = font_width[*text] * rescale;
		float width = height * width_to_height;
		glBindTexture(GL_TEXTURE_2D, font_map[*text]);
		glBegin(GL_QUADS);
		glTexCoord2f(0, 1.0 * rescale);
		glVertex2f(x, y);
		glTexCoord2f(0, 1 - 2.0 * rescale);
		glVertex2f(x, y+height);
		glTexCoord2f(1, 1 - 2.0 * rescale);
		glVertex2f(x+width, y+height);
		glTexCoord2f(1, 1.0 * rescale);
		glVertex2f(x+width, y);
		glEnd();
		x += width;
		text++;
	}
	glDisable(GL_TEXTURE_2D);
	glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);
}

extern "C" EXPORT void set_color(float r, float g, float b, float a) {
	glColor4f(r, g, b, a);
}

extern "C" EXPORT void translate(float x, float y, float z) {
	glTranslatef(x, y, z);
}

extern "C" EXPORT void draw_triangle(float* coords, int texture) {
//	glColor4f(1.0f,1.0f,1.0f,1.0f);
//	if (texture != -1)
//		glBindTexture(GL_TEXTURE_2D, textures[texture]);
	glBegin(GL_TRIANGLES);
//	if (texture != -1)
//		glTexCoord2f(coords[9], coords[10]);
	glVertex3f(coords[0], coords[1], coords[2]);
//	if (texture != -1)
//		glTexCoord2f(coords[11], coords[12]);
	glVertex3f(coords[3], coords[4], coords[5]);
//	if (texture != -1)
//		glTexCoord2f(coords[13], coords[14]);
	glVertex3f(coords[6], coords[7], coords[8]);
	glEnd();
}

extern "C" EXPORT void draw_sphere(float x, float y, float z, float radius) {
	// FIXME: Eh... let's do a cube for now.
	float bounds[6] = {x-radius, x+radius, y-radius, y+radius, z-radius, z+radius};
	draw_box(bounds, -1);
}

extern "C" EXPORT void draw_triangles(int count, float* coords, float* texture_coords, int texture_index) {
	//glDisable(GL_TEXTURE_2D);
	if (texture_index != -1) {
		glEnable(GL_TEXTURE_2D);
		glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
		glBindTexture(GL_TEXTURE_2D, texture_index);
	}
	glBegin(GL_TRIANGLES);
	for (int i = 0; i < count; i++) {
//		for (int vertex = 0; vertex < 3; vertex++) {
		glTexCoord2f(texture_coords[0], texture_coords[1]);
		glVertex3f(coords[0], coords[1], coords[2]);
		glTexCoord2f(texture_coords[2], texture_coords[3]);
		glVertex3f(coords[3], coords[4], coords[5]);
		glTexCoord2f(texture_coords[4], texture_coords[5]);
		glVertex3f(coords[6], coords[7], coords[8]);
		texture_coords += 6;
		coords += 9;
//		}
	}
	glEnd();
	if (texture_index != -1) {
		glDisable(GL_TEXTURE_2D);
	}
}

extern "C" EXPORT void draw_box(float* coords, int texture_index) {
	float xmin = coords[0], xmax = coords[1];
	float ymin = coords[2], ymax = coords[3];
	float zmin = coords[4], zmax = coords[5];
	if (texture_index != -1) {
		glEnable(GL_TEXTURE_2D);
		glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
		glBindTexture(GL_TEXTURE_2D, texture_index);
	}
	// Draw the eight walls.
	glBegin(GL_QUADS);
	// Top face.
	#define TEXTURE(x, y) \
		glTexCoord2f(x/5.0, y/5.0);
	TEXTURE     (xmin, ymin      );
	glVertex3f  (xmin, ymin, zmax);
	TEXTURE     (xmax, ymin      );
	glVertex3f  (xmax, ymin, zmax);
	TEXTURE     (xmax, ymax      );
	glVertex3f  (xmax, ymax, zmax);
	TEXTURE     (xmin, ymax      );
	glVertex3f  (xmin, ymax, zmax);
	// Bottom face.
	TEXTURE     (xmin, ymax);
	glVertex3f  (xmin, ymax, zmin);
	TEXTURE     (xmax, ymax);
	glVertex3f  (xmax, ymax, zmin);
	TEXTURE     (xmax, ymin);
	glVertex3f  (xmax, ymin, zmin);
	TEXTURE     (xmin, ymin);
	glVertex3f  (xmin, ymin, zmin);
	// Left face.
	TEXTURE     (      ymin, zmin);
	glVertex3f  (xmin, ymin, zmin);
	TEXTURE     (      ymin, zmax);
	glVertex3f  (xmin, ymin, zmax);
	TEXTURE     (      ymax, zmax);
	glVertex3f  (xmin, ymax, zmax);
	TEXTURE     (      ymax, zmin);
	glVertex3f  (xmin, ymax, zmin);
	// Right face.
	TEXTURE     (      ymax, zmin);
	glVertex3f  (xmax, ymax, zmin);
	TEXTURE     (      ymax, zmax);
	glVertex3f  (xmax, ymax, zmax);
	TEXTURE     (      ymin, zmax);
	glVertex3f  (xmax, ymin, zmax);
	TEXTURE     (      ymin, zmin);
	glVertex3f  (xmax, ymin, zmin);
	// Front face.
	TEXTURE     (xmin,       zmin);
	glVertex3f  (xmin, ymin, zmin);
	TEXTURE     (xmax,       zmin);
	glVertex3f  (xmax, ymin, zmin);
	TEXTURE     (xmax,       zmax);
	glVertex3f  (xmax, ymin, zmax);
	TEXTURE     (xmin,       zmax);
	glVertex3f  (xmin, ymin, zmax);
	// Rear face.
	TEXTURE     (xmin,       zmax);
	glVertex3f  (xmin, ymax, zmax);
	TEXTURE     (xmax,       zmax);
	glVertex3f  (xmax, ymax, zmax);
	TEXTURE     (xmax,       zmin);
	glVertex3f  (xmax, ymax, zmin);
	TEXTURE     (xmin,       zmin);
	glVertex3f  (xmin, ymax, zmin);
	glEnd();
	if (texture_index != -1) {
		glDisable(GL_TEXTURE_2D);
//		glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);
//		glBindTexture(GL_TEXTURE_2D, texture_index);
	}
}

extern "C" EXPORT void warp_mouse(int x, int y) {
	SDL_WarpMouse(x, y);
}

extern "C" EXPORT int get_event() {
	SDL_Event ev;
	while (SDL_PollEvent(&ev)) {
		switch (ev.type) {
			case SDL_QUIT:
				last_event.type = 1;
				return 1;
			case SDL_MOUSEBUTTONDOWN:
				last_event.type = 2;
				last_event.button = ev.button.button;
				return 1;
			case SDL_MOUSEBUTTONUP:
				last_event.type = 3;
				last_event.button = ev.button.button;
				return 1;
			case SDL_MOUSEMOTION:
				mouse_x = ev.motion.x;
				mouse_y = ev.motion.y;
				break;
			case SDL_KEYDOWN:
				last_event.type = 4;
				last_event.key = ev.key.keysym.sym;
				return 1;
			case SDL_KEYUP:
				last_event.type = 5;
				last_event.key = ev.key.keysym.sym;
				return 1;
		}
	}
	return 0;
}

/*
extern "C" EXPORT void play_music(char* path) {
  Mix_PlayMusic(Mix_LoadMUS(path), -1); // -1 loops the music forever
}

extern "C" EXPORT void play_sound(char* path) {
  Mix_PlayMusic(Mix_LoadMUS(path), 0); // 0, plays once. TODO: Load all sfx beforehand to avoid wasting memory
}
*/

extern "C" EXPORT int graphics_init(int fullscreen) {
	if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO) < 0)
		return 0;
	const SDL_VideoInfo* info = SDL_GetVideoInfo();
	if (info == NULL) {
		cerr << "Unable to get video info: " << SDL_GetError() << endl;
		return 0;
	}
	screen_width  = info->current_w;
	screen_height = info->current_h;
	if (not fullscreen) {
		screen_width  = 1200;
		screen_height = 480;
	}
	int video_flags = 0;
	video_flags  = SDL_OPENGL;
	video_flags |= SDL_GL_DOUBLEBUFFER;
	video_flags |= SDL_HWPALETTE;
	if (fullscreen)
		video_flags |= SDL_FULLSCREEN;
	SDL_GL_SetAttribute(SDL_GL_SWAP_CONTROL, 1);
	if (!(screen = SDL_SetVideoMode(screen_width, screen_height, 32, video_flags))) {
		cerr << "Couldn't SetVideoMode: " << SDL_GetError() << endl;
		SDL_Quit();
		return 0;
	}
	if (fullscreen)
		SDL_ShowCursor(SDL_DISABLE);
	glShadeModel(GL_SMOOTH);
	glEnable(GL_CULL_FACE);
//	glEnable(GL_MULTISAMPLE);
//	glDisable(GL_CULL_FACE);
	glEnable(GL_COLOR_MATERIAL);
	glDisable(GL_TEXTURE_2D);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
	glDisable(GL_DEPTH_TEST);
	glDepthFunc(GL_LESS);
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
//	glClearColor(0.5, 0.6, 1.0, 1.0);
	glClearColor(0.1, 0.1, 0.1, 1.0);

	// Create the default orthogonal projection matrix.
//	glMatrixMode(GL_PROJECTION);
//	glLoadIdentity();
//	glOrtho(0, screen_width, screen_height, 0, 0, 1);

	glMatrixMode(GL_PROJECTION);

	// Reset The Projection Matrix
	glLoadIdentity();

	// Calculate The Aspect Ratio Of The Window
//	gluPerspective(70.0f, screen_width / (double)screen_height, 0.05f, 1000.0f);
	GLdouble fovY = 70.0, aspect = screen_width / (double)screen_height;
	GLdouble zNear = 0.05, zFar = 1000;
	GLdouble fH = tan( fovY / 360 * M_PI ) * zNear;
	GLdouble fW = fH * aspect;
	glFrustum( -fW, fW, -fH, fH, zNear, zFar );

	glMatrixMode(GL_MODELVIEW);

	// Set up light 1
#ifdef USE_LIGHTS
	glEnable(GL_LIGHTING);

	glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER, GL_TRUE);

	GLfloat lmodel_ambient[] = { 0.2, 0.2, 0.2, 1.0 };
	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, lmodel_ambient);

	glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
	glEnable(GL_COLOR_MATERIAL);

	glLightfv(GL_LIGHT0, GL_AMBIENT,  LightAmbient);  // Add lighting -- Ambient
	glLightfv(GL_LIGHT0, GL_DIFFUSE,  LightDiffuse);  // Add lighting -- Diffuse
	glLightfv(GL_LIGHT0, GL_POSITION, LightPosition); // Set light position
	glEnable(GL_LIGHT0);							  // Turn light 1 on
#endif

	// Load up the font.
	for (int ii=0; ii<95; ii++) {
		int character = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789!\"#$%&\'()*+,-./:;<=>?@[\\]^_`{|}~ "[ii];
		char buf[32];
		snprintf(buf, sizeof(buf), "bitfont/%i.png", character);
		int width_height[2];
		font_map[character] = png_texture_load(buf, width_height, 1);
		font_width[character] = width_height[0];
		font_height[character] = width_height[1];
	}

/*
        // Sound init shamelessly pasted from http://content.gpwiki.org/
        int audio_rate = 22050;
        Uint16 audio_format = AUDIO_S16SYS;
        int audio_channels = 2;
        int audio_buffers = 4096;

        if(Mix_OpenAudio(audio_rate, audio_format, audio_channels, audio_buffers) != 0) {
          fprintf(stderr, "Unable to initialize audio: %s\n", Mix_GetError());
          exit(1);
        }
*/
}

extern "C" EXPORT int graphics_close() {
	SDL_Quit();
}

extern "C" EXPORT void gettimeofday_wrapper(long long* result) {
	struct timeval tv;
	gettimeofday(&tv, NULL);
	result[0] = tv.tv_sec;
	result[1] = tv.tv_usec;
}

