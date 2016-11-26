#include "header.h"

const int MARKERNUM = 2;

//cm����
const int MARKER_HEIGHT = 8;
const int MARKER_WIDTH = 6;

const int zNear = 1.0;
const int zFar = 1000000;

const int FRAME_WIDTH = 640;
const int FRAME_HEIGHT = 480;

const int objectSize = 2;
const double PI = 3.14159265359;

const std::string nmarker[] = { "./picture/nexen.png", "./picture/pringles.jpg" };

void init(void);
void convertFromCaemraToOpenGLProjection(double* mGL);
bool calculatePoseFromH(const cv::Mat& H, cv::Mat& R, cv::Mat& T);
void processVideoCapture(void);
void display(void);
void idle(void);
void reshape(int w, int h);

void printSolidTeapot();
void printWireTeapot();
void printSolidCube();
void printWireCube();