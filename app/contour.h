//struct colour8_t convertDepthToColour(uint16_t);

/** colour8_t 
	Represents pixel colour with 8 bit RGB depth
**/
typedef struct {
	uint8_t red;
	uint8_t green;
	uint8_t blue;
} colour8_t;

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

//typedef struct colour8_t;
