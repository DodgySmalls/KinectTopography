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
#define RT_DEBUG
#define INIT_DEBUG

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
#define DEFAULT_INIT_PATH "colour.init"
#define DEFAULT_LINE_BUFFER_BYTES 100
#define DEFAULT_COLOUR_CAP 16

#define DEPTH_CB_X 640
#define DEPTH_CB_Y 480
#define DEPTH_CB_RANGE 2048

/** colour8_t 
	A glob representing a pixel colour with 8 bit RGB depth
**/
typedef struct {
	uint8_t red;
	uint8_t green;
	uint8_t blue;
} colour8_t;


/**
		function headers
**/
//struct colour8_t convertDepthToColour(uint16_t);

void depthCB(freenect_device*, void*, uint32_t);
void * freenectThreadfunc(void*);
void depthCallback(freenect_device *, void *, uint32_t);
void drawGLScene();
void resizeGLScene(int, int);
void launchGL(int, char**);
void keyPressed(unsigned char, int, int);
void * verifyMemory(void*);
void initKinect(int, char**);
void generateSpectrum(colour8_t *, int, colour8_t *, float *, int);
void initSpectrum();


/** 
		global variables 
**/
GLuint gl_depth_tex;
int window;
colour8_t * depth_front;
colour8_t * depth_mid;
colour8_t * frame_clone;

volatile int die = 0;
int got_depth = 0;
pthread_mutex_t gl_backbuf_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t gl_frame_cond = PTHREAD_COND_INITIALIZER;
pthread_t freenect_thread;

freenect_context *f_ctx;
freenect_device *f_dev;
int freenect_led;
freenect_video_format requested_format = FREENECT_VIDEO_RGB;
freenect_video_format current_format = FREENECT_VIDEO_RGB;

colour8_t spectrum[DEPTH_CB_RANGE];

int num_colours;
colour8_t colours[DEFAULT_COLOUR_CAP];
float colour_weight[DEFAULT_COLOUR_CAP];

uint8_t * ghettoContourMasks;

uint16_t contour_range = DEPTH_CB_RANGE;
int contour_start = 1600;
int num_contour_masks = 45;
int contour_mask_range;

int main(int argc, char ** argv){
	//allocate blocks of heap memory for frames
	depth_front = (colour8_t *)verifyMemory(malloc(DEPTH_CB_X * DEPTH_CB_Y * 3));
	depth_mid = (colour8_t *)verifyMemory(malloc(DEPTH_CB_X * DEPTH_CB_Y * 3));
	frame_clone = (colour8_t *)verifyMemory(malloc(DEPTH_CB_X * DEPTH_CB_Y * 3));

	//spectrum = (colour8_t *)verifyMemory(malloc(DEPTH_CB_RANGE * 3));
	initSpectrum();

	ghettoContourMasks = (uint8_t *)verifyMemory(malloc(DEPTH_CB_X * DEPTH_CB_Y*sizeof(uint8_t)));
	contour_mask_range = contour_range / num_contour_masks;

	initKinect(argc, argv);

	if (pthread_create(&freenect_thread, NULL, freenectThreadfunc, NULL) != 0) {
		fprintf(stderr, "pthread_create failed\n");
		freenect_shutdown(f_ctx);
		return 1;
	}

	launchGL(argc, argv);

	return 0;
}

void initSpectrum() {
	FILE * colour_init;

	//init colours from file
	colour_init = fopen(DEFAULT_INIT_PATH, "r");
	num_colours = 0;
	while(num_colours < DEFAULT_COLOUR_CAP) {
		int inp1;
		int inp2;
		int inp3;

		if(fscanf(colour_init, "%f %d %d %d", &(colour_weight[num_colours]), &inp1, &inp2, &inp3) > 0) {
			printf("Got colour %f - %d|%d|%d\n", colour_weight[num_colours], inp1, inp2, inp3);
			colours[num_colours].red = (uint8_t)inp1;
			colours[num_colours].green = (uint8_t)inp2;
			colours[num_colours].blue = (uint8_t)inp3;
			num_colours++;
		} else {
			break;
		}
	}
	fclose(colour_init);

	generateSpectrum(&spectrum, DEPTH_CB_RANGE, &colours, &colour_weight, num_colours);
}

void initKinect(int cargc, char ** cargv) {
	int dev_count;
	int user_device_number;

	if (freenect_init(&f_ctx, NULL) < 0) {
		fprintf(stderr, "freenect_init() failed\n");
		exit(1);
	}

	dev_count = freenect_num_devices (f_ctx);
	
	#ifdef RT_DEBUG
	printf("Number of devices found: %d\n", dev_count);
	#endif

	freenect_set_log_level(f_ctx, FREENECT_LOG_DEBUG);
    freenect_select_subdevices(f_ctx, (freenect_device_flags)(FREENECT_DEVICE_MOTOR | FREENECT_DEVICE_CAMERA)); 

	/* device selection */
	user_device_number = 0;
	if (cargc > 1 && dev_count > 1) {
		user_device_number = atoi(cargv[1]);
	} else if (dev_count < 1) {
		fprintf(stderr, "No devices detected");
		freenect_shutdown(f_ctx);
		exit(1);
	}

	if (freenect_open_device(f_ctx, &f_dev, user_device_number) < 0) {
		fprintf(stderr, "Could not open device\n");
		freenect_shutdown(f_ctx);
		exit(1);
	}
}

/** initializes and launches a glut window bound to the kinect's depth callback **/
void launchGL(int g_argc, char ** g_argv) {
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


void *freenectThreadfunc(void *arg) {
	int tick = 0;

	freenect_set_led(f_dev,LED_GREEN);
	freenect_set_depth_callback(f_dev, depthCB);
	//freenect_set_video_mode(f_dev, freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, current_format));
	freenect_set_depth_mode(f_dev, freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_DEPTH_11BIT));

	freenect_start_depth(f_dev);

	while (!die && freenect_process_events(f_ctx) >= 0) {
		//twiddle thumbs
		tick++;
	}
	
	#ifdef RT_DEBUG
	printf("<freenectThreadfunc()> shutting down streams!\n");
	#endif
	
	freenect_stop_depth(f_dev);
	freenect_stop_video(f_dev);

	freenect_close_device(f_dev);
	freenect_shutdown(f_ctx);
	#ifdef RT_DEBUG
	printf("<freenectThreadfunc()> done!\n");
	#endif
	return NULL;
}

//draws the frame dictated by the kinect's depth callback into depth_mid
void depthCB(freenect_device *dev, void *v_depth, uint32_t timestamp) {
	int i;
	int p;
	uint16_t *depth = (uint16_t*)v_depth;

	pthread_mutex_lock(&gl_backbuf_mutex);
	
	for(i=0;i<DEPTH_CB_X * DEPTH_CB_Y;i++) {
		//prime each pixel
		depth_mid[i] = spectrum[depth[i]];
		//prime the contourmasks
		/*if(depth[i] < contour_start || depth[i] > contour_range + contour_start) {
			ghettoContourMasks[i] = 255;
		} else {
			ghettoContourMasks[i] = (uint8_t)(depth[i]-contour_start / contour_mask_range);
		}*/
		ghettoContourMasks[i] = (uint8_t)(depth[i] / contour_mask_range);
	}
	
	for(i=1; i<DEPTH_CB_Y-1; i++) {
		for(int j=1; j<DEPTH_CB_X-1; j++){
			p=(i*DEPTH_CB_X) + j;
			assert(p-1 > 0);
			if(!(ghettoContourMasks[p]==ghettoContourMasks[p-1])){
				depth_mid[p].red = 0;
				depth_mid[p].green = 0;
				depth_mid[p].blue = 0;
			}
			assert(p+1 < DEPTH_CB_X * DEPTH_CB_Y);
			if(!(ghettoContourMasks[p]==ghettoContourMasks[p+1])){
				depth_mid[p].red = 0;
				depth_mid[p].green = 0;
				depth_mid[p].blue = 0;
			}
			assert(p+DEPTH_CB_X < DEPTH_CB_X * DEPTH_CB_Y);
			if(!(ghettoContourMasks[p]==ghettoContourMasks[p+DEPTH_CB_X])){
				depth_mid[p].red = 0;
				depth_mid[p].green = 0;
				depth_mid[p].blue = 0;
			}
			assert(p-DEPTH_CB_X > 0);
			if(!(ghettoContourMasks[p]==ghettoContourMasks[p-DEPTH_CB_X])){
				depth_mid[p].red = 0;
				depth_mid[p].green = 0;
				depth_mid[p].blue = 0;
			}
		}
	}

	got_depth++;
	pthread_cond_signal(&gl_frame_cond);
	pthread_mutex_unlock(&gl_backbuf_mutex);
}

//Attemps to pull data from the kinect's depth camera and draw the next frame using it.
void drawGLScene() {

	pthread_mutex_lock(&gl_backbuf_mutex);

	// When using YUV_RGB mode, RGB frames only arrive at 15Hz, so we shouldn't force them to draw in lock-step.
	// However, this is CPU/GPU intensive when we are receiving frames in lockstep.
	if (current_format == FREENECT_VIDEO_YUV_RGB) {
		while (!got_depth) {
			pthread_cond_wait(&gl_frame_cond, &gl_backbuf_mutex);
		}
	}

	if (requested_format != current_format) {
		pthread_mutex_unlock(&gl_backbuf_mutex);
		return;
	}

	colour8_t *tmp;

	if (got_depth) {
		tmp = depth_front;
		depth_front = depth_mid;
		depth_mid = tmp;
		got_depth = 0;
	}


	pthread_mutex_unlock(&gl_backbuf_mutex);
	glBindTexture(GL_TEXTURE_2D, gl_depth_tex);
	glTexImage2D(GL_TEXTURE_2D, 0, 3, 640, 480, 0, GL_RGB, GL_UNSIGNED_BYTE, (uint8_t *)depth_front);

	int camera_angle = 0;
	glLoadIdentity();
	
	glPushMatrix();
	glTranslatef((640.0/2.0),(480.0/2.0) ,0.0);
	glRotatef(camera_angle, 0.0, 0.0, 1.0);
	glTranslatef(-(640.0/2.0),-(480.0/2.0) ,0.0);
	glBegin(GL_TRIANGLE_FAN);
	glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
	glTexCoord2f(0, 1); glVertex3f(0,0,1.0);
	glTexCoord2f(1, 1); glVertex3f(640,0,1.0);
	glTexCoord2f(1, 0); glVertex3f(640,480,1.0);
	glTexCoord2f(0, 0); glVertex3f(0,480,1.0);
	glEnd();
	glPopMatrix();
	
	glPushMatrix();
	glTranslatef(640+(640.0/2.0),(480.0/2.0) ,0.0);
	glRotatef(camera_angle, 0.0, 0.0, 1.0);
	glTranslatef(-(640+(640.0/2.0)),-(480.0/2.0) ,0.0);

	glBegin(GL_TRIANGLE_FAN);
	glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
	glTexCoord2f(0, 1); glVertex3f(640,0,0);
	glTexCoord2f(1, 1); glVertex3f(1280,0,0);
	glTexCoord2f(1, 0); glVertex3f(1280,480,0);
	glTexCoord2f(0, 0); glVertex3f(640,480,0);
	glEnd();
	glPopMatrix();

	glutSwapBuffers();
}

//resizes the glfuncs to match the new viewport size
void resizeGLScene(int Width, int Height) {
	glViewport(0,0,Width,Height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho (0, 640, 0, 480, -5.0f, 5.0f);
	glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

//modifies the active range by changing the upper (0) or lower (1) bound
//depth values increase as distance from the camera increases (ie. min->0 max->2048)
//returns the size of the new active range
int resizeRange(int * rbottom, int * rtop, int rdir, int dist) {
	if(rdir == 0) {
		*rtop += dist;
	} else {
		*rbottom += dist;
	}

	//ensure values stay in bounds (DEPTH_CB_RANGE default is 2048)
	if(*rtop > DEPTH_CB_RANGE) { 
		*rtop = DEPTH_CB_RANGE;
	} else if(*rtop < 0) {
		*rtop = 0;
	}

	if(*rbottom > DEPTH_CB_RANGE) {
		*rbottom = DEPTH_CB_RANGE;
	} else if(*rbottom < 0) {
		*rbottom = 0;
	}

	return (*rbottom - *rtop);
}

//generate a spectrum representing linear gradients between each breakpoint colour
void generateSpectrum(colour8_t * dump, int size, colour8_t * breakpoints, float * colour_weights, int num_breakpoints) {
	#ifdef INIT_DEBUG
	printf("generateSpectrum(*dump, %d, *breakpoints, floatlist, %d)\n", size, num_breakpoints);
	printf("colour_weights[x], 0=%f, 1=%f, 2=%f, 3=%f\n", colour_weights[0], colour_weights[1], colour_weights[2], colour_weights[3]);
	#endif
	//used to determine what %slice of the spectrum each gradient receives
	float total_weight = 0;
	//used to avoid missing integers due to imprecision, the weight algorithm is imperfect
	int colourpos = 0;
	//temp vars for readability
	int r,g,b;
	float cfr,cto;

	colour8_t colour_from;
	colour8_t colour_to;

	if (num_breakpoints <= 1){
		//two few colours, at least two colours (and hence one gradient weight) are required, generate default spectrum in this case.
		fprintf(stderr, "too few breakpoints specified, generating default spectrum\n");
		fprintf(stderr, "code incomplete, exiting\n");
		//generate 255-0-0 _1.0_ 0-255-0 _1.0_ 0-0-255
		exit(77);
	}
	for(int i=0;i<num_breakpoints-1;i++) {
		total_weight+=colour_weights[i];
	}
	
	for(int c=0;c<num_breakpoints-1;c++) {
		int colour_dist = (colour_weights[c] / total_weight) * size; //the number of discrete blocks which will be subject to the gradient from colour[c] -> colour[c+1]
		colour_to = breakpoints[c+1];
		colour_from = breakpoints[c];

		//legacy error handling and algorithm imprecision, usually several memory slots remain untouched, likely due to truncating floats
		if(colourpos + colour_dist > size || c == num_breakpoints-2) {
			colour_dist = size - colourpos;
		}

		for(int i=0; i < colour_dist;i++) {
			
			#ifdef INIT_DEBUG 
			assert(colourpos<size);
			#endif

			cto = ((float)i) / ((float)colour_dist);
			cfr = 1-( ((float)i) / ((float)colour_dist) );

			r = (int) ((cto * ((float)colour_to.red)) + (cfr * ((float)colour_from.red)));
			g = (int) ((cto * ((float)colour_to.green)) + (cfr * ((float)colour_from.green)));
			b = (int) ((cto * ((float)colour_to.blue)) + (cfr * ((float)colour_from.blue)));

			(dump[colourpos]).red = (uint8_t)r;
			(dump[colourpos]).green = (uint8_t)g;
			(dump[colourpos]).blue = (uint8_t)b;
			colourpos++;
		}
	}

	#ifdef INIT_DEBUG
	for(int i=0;i<size;i++){
		r = (int)((dump[i]).red); 
		g = (int)((dump[i]).green);
		b = (int)((dump[i]).blue);

		printf("colour[%d] = (%d %d %d)\n", i, (int)r, (int)g, b);
	}
	#endif
}

void keyPressed(unsigned char key, int x, int y) {
	if (key == 27) {
		die = 1;
		pthread_join(freenect_thread, NULL);
		glutDestroyWindow(window);
		free(depth_mid);
		free(depth_front);
		free(frame_clone);
		exit(0);
	}
	return;
}

//graceful exit from failed heap allocation
void * verifyMemory(void * p) {
	if(p == NULL) {
		fprintf(stderr, "Failed to allocate memory, exiting");
		exit(0);
	}
	return p;
}