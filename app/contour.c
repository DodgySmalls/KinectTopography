/*
 * This file is part of the OpenKinect Project. http://www.openkinect.org
 *
 * Copyright (c) 2010 individual OpenKinect contributors. See the CONTRIB file
 * for details.
 *
 * This code is licensed to you under the terms of the Apache License, version
 * 2.0, or, at your option, the terms of the GNU General Public License,
 * version 2.0. See the APACHE20 and GPL2 files for the text of the licenses,
 * or the following URLs:
 * http://www.apache.org/licenses/LICENSE-2.0
 * http://www.gnu.org/licenses/gpl-2.0.txt
 *
 * If you redistribute this file in source form, modified or unmodified, you
 * may:
 *   1) Leave this header intact and distribute it under the same terms,
 *      accompanying it with the APACHE20 and GPL20 files, or
 *   2) Delete the Apache 2.0 clause and accompany it with the GPL2 file, or
 *   3) Delete the GPL v2 clause and accompany it with the APACHE20 file
 * In all cases you must keep the copyright notice intact and include a copy
 * of the CONTRIB file.
 *
 * Binary distributions must follow the binary distribution requirements of
 * either License.
 */

//inline defines simpler than arguments with cmake
#define GLOBAL_DEBUG
#define GL_DEBUG

#ifdef GLOBAL_DEBUG
#include <assert.h>
#include <inttypes.h>
#endif

//std
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

//opengl/openkinect
#include "libfreenect.h"
#include <pthread.h>
#if defined(__APPLE__)
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#define _USE_MATH_DEFINES
#include <math.h>

#define DEFAULT_WINDOW_X 640
#define DEFAULT_WINDOW_Y 480
#define DEFAULT_WINDOW_TITLE "OSX Contour"

#define DEPTH_CB_X 640
#define DEPTH_CB_Y 480

/**
		function headers
**/
struct colour8_t convertDepthToColour(uint16_t);

void depthCallback(freenect_device *, void *, uint32_t);
void drawGLScene();
void resizeGLScene(int, int);
void launchGL(int, char **);
void keyPressed(unsigned char, int, int);
void ** verifyMemory(void **);
//void initKinect();

/** colour8_t 
	A glob representing a pixel colour with 8 bit RGB depth
**/
typedef struct {
	uint8_t red;
	uint8_t green;
	uint8_t blue;
} colour8_t;

/** 
		global variables 
**/
GLuint gl_depth_tex;
int window;

colour8_t * depth_front;
colour8_t * depth_mid;
colour8_t * frame_clone;

int main(int argc, char ** argv){
	//allocate blocks of heap memory for frames
	depth_front = (colour8_t *)malloc(DEPTH_CB_X * DEPTH_CB_Y * 3);
	depth_mid = (colour8_t *)malloc(DEPTH_CB_X * DEPTH_CB_Y * 3);
	frame_clone = (colour8_t *)malloc(DEPTH_CB_X * DEPTH_CB_Y * 3);

	//initKinect();
	launchGL(argc, argv);
	return 0;
}

/** initializes and launches a glut window bound to the kinect's depth callback **/
void launchGL(int g_argc, char ** g_argv) {
		#ifdef GL_DEBUG
		printf("GL thread\n");
		#endif

	glutInit(&g_argc, g_argv);
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_ALPHA | GLUT_DEPTH | GLUT_MULTISAMPLE);
	glutInitWindowSize(DEFAULT_WINDOW_X, DEFAULT_WINDOW_Y);
	glutInitWindowPosition(0, 0);

	window = glutCreateWindow(DEFAULT_WINDOW_TITLE);

	glutDisplayFunc(&drawGLScene);
	glutIdleFunc(&drawGLScene);
	glutReshapeFunc(&resizeGLScene);
	glutKeyboardFunc(&keyPressed);

	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
	glDisable(GL_DEPTH_TEST);
	glDisable(GL_BLEND);
	glDisable(GL_ALPHA_TEST);
	glEnable(GL_TEXTURE_2D);
	glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glShadeModel(GL_FLAT);
	glEnable(GL_MULTISAMPLE);

	glGenTextures(1, &gl_depth_tex);
	glBindTexture(GL_TEXTURE_2D, gl_depth_tex);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	resizeGLScene(DEFAULT_WINDOW_X, DEFAULT_WINDOW_Y);

	glutMainLoop();
}

//Attemps to pull data from the kinect's depth camera and draw the next frame using it.
void drawGLScene() {
	
	glutSwapBuffers();
}

void resizeGLScene(int Width, int Height) {
	glViewport(0,0,Width,Height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho (0, 1280, 0, 480, -5.0f, 5.0f);
	glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

void keyPressed(unsigned char key, int x, int y) {
	if (key == 27) {
		//kinect cleanup disabled during GL testing
		/*die = 1;
		pthread_join(freenect_thread, NULL);
		glutDestroyWindow(window);
		free(depth_mid);
		free(depth_front);
		// Not pthread_exit because OSX leaves a thread lying around and doesn't exit
		*/
		exit(0);
	}
	return;
}

//graceful exit from failed heap allocation
void ** verifyMemory(void ** p) {
	if(*p == NULL) {
		fprintf(stderr, "Failed to allocate memory, exiting");
		exit(0);
	}
	return p;
}