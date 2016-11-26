#include "header.h"

// 주먹 범위
const int rock_low = 14;
const int rock_high = 20;

// 가위 범위
const int scissors_low = 21;
const int scissors_high = 29;

//보 범위
const int paper_low = 31;
const int paper_high = 41;

enum rsp { rock, scissors, paper, none };

int checkNoise(int * noise, int noise_index, int pixel);
int checkPixel(Mat pic);
rsp check_rsp(int pixel, int noise);
void fillMat(Mat * mat, int n);
rsp image_processing(Mat camFrame, Rect box);