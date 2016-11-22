#include <iostream>
#include <fstream>
#include <cmath>

#include "opencv2\highgui\highgui.hpp"
#include "opencv2\opencv.hpp"

#pragma comment(lib, "opencv_world300d.lib")

const double PI = 3.14159265;

using namespace std;
using namespace cv;

bool bLBDown = false; // 마우스 버튼 눌렀는지 체크
bool checkDrag; // 드래그가 이루어졌는지 체크
CvRect box; // 드래그로 그린 박스

// 사각형 그리기
void draw_box(IplImage* img, CvRect rect)
{
	cvRectangle(img, cvPoint(rect.x, rect.y),
		cvPoint(rect.x + rect.width, rect.y + rect.height),
		cvScalar(0xff, 0x00, 0x00));
}

// 마우스 드래그
void on_mouse(int event, int x, int y, int flag, void* params)
{
	IplImage* image = (IplImage*)params;

	if (event == CV_EVENT_LBUTTONDOWN){ // 왼쪽 버튼 눌렀을 시, 박스 초기화
		bLBDown = true;
		box = cvRect(x, y, 0, 0);
	}
	else if (event == CV_EVENT_LBUTTONUP){ // 왼쪽 버튼 눌렀다가 뗐을 때, 박스의 넓이, 높이를 설정한다.
		bLBDown = false;
		checkDrag = true;
		if (box.width < 0)
		{
			box.x += box.width;
			box.width *= -1;
		}
		if (box.height < 0)
		{
			box.y += box.height;
			box.height *= -1;
		}
		draw_box(image, box);
	} 
	else if (event == CV_EVENT_MOUSEMOVE && bLBDown){ // 드래그 중에는 박스의 넓이, 높이를 갱신한다.
		box.width = x - box.x;
		box.height = y - box.y;
	}
}

// 이미지 복사
Mat copyMat(Mat source)
{
	// source의 Mat을 result로 복사하는 작업
	// opencv에 이미 구현이 되어있는 작업이다.
	// source.copyTo(result);
	Mat result = Mat::zeros(source.size(), source.type());
	for (int i = 0; i < source.cols; i++){
		for (int j = 0; j < source.rows; j++){
			result.at<Vec3b>(j, i) = source.at<Vec3b>(j, i);
		}
	}

	return result;
}

// 박스내 이미지 복사
Mat copyBoxMat(Mat source)
{
	return source(box);
}

// y축반전
Mat yReflecting(Mat source)
{
	Mat result = copyMat(source);

	for (int i = 0; i < box.width; i++){
		for (int j = 0; j < box.height; j++){
			result.at<Vec3b>((box.y + j), (box.x + i)) = source.at<Vec3b>(box.y + j, (box.width + box.x - 1) - i);
		}
	}

	return result;
}

// x축반전
Mat xReflecting(Mat source)
{
	Mat result = copyMat(source);

	for (int i = 0; i < box.width; i++){
		for (int j = 0; j < box.height; j++){
			result.at<Vec3b>((box.y + j), (box.x + i)) = source.at<Vec3b>((box.height + box.y - 1) - j, (box.x + i));
		}
	}

	return result;
}

// 회전
Mat rotating(Mat source, double degree)
{
	Mat result = copyMat(source);

	int x0 = box.x + (box.width / 2);
	int y0 = box.y + (box.height / 2);
	double cosd = cos(degree*PI / 180);
	double sind = sin(degree*PI / 180);
	
	// 원본에 덮어씌우는 부분으로 인해 왼쪽 90도, 오른쪽 90도만 가능
	for (int i = 0; i < box.width; i++){
		for (int j = 0; j < box.height; j++){
			int x1 = (box.x + i);
			int y1 = (box.y + j);
			int x = ((cosd * (x1 - x0)) - (sind * (y1 - y0)) + x0);
			int y = ((sind * (x1 - x0)) - (cosd * (y1 - y0)) + y0);
			result.at<Vec3b>(y, x) = source.at<Vec3b>((box.y + j), (box.x + i));
		}
	}

	return result;
}

// 확대
Mat scaling(Mat source, Mat boxMat, double scale)
{
	Mat result = copyMat(source);
	Mat scaleBoxMat;

	// 사각형 안의 Mat의 크기를 scale배 늘린다.
	int boxWidth = (int)(boxMat.size().width * scale);
	int boxHeight = (int)(boxMat.size().height * scale);
	cv::resize(boxMat, scaleBoxMat, Size(boxWidth, boxHeight));

	// 붙여넣을 때 시작 위치 정보를 갱신한다.
	int x = box.x - (box.width / 2);
	int y = box.y - (box.height / 2);

	for (int i = 0; i < boxWidth; i++){
		for (int j = 0; j < boxHeight; j++){
			result.at<Vec3b>((y + j), (x + i)) = scaleBoxMat.at<Vec3b>(j, i);
		}
	}

	return result;
}

int main()
{
	IplImage copy;
	IplImage * resultImage;
	Mat resultMat, xReflectMat, yReflectMat, leftRotateMat, scaleMat, boxMat;

	// 이미지 불러오기
    Mat gMatImage = imread("./picture/pic.jpg", 1);

	// Mat 이미지를 IplImage 로 복사한다.
	copy = gMatImage;
	resultImage = &copy;


	checkDrag = false;
	namedWindow("image");
	setMouseCallback("image", on_mouse, resultImage);
	cvShowImage("image", resultImage);

	//드래그 대기
	while (!checkDrag){
		waitKey(100);
	}
	cvShowImage("image", resultImage);

	//사각형 추가된 사진 저장
	resultMat = cvarrToMat(resultImage);
	boxMat = copyBoxMat(resultMat);

	cout << box.x << ' ' << box.y << ' ' << box.width << ' ' << box.height << endl;

	yReflectMat = yReflecting(resultMat); // y축 반전
	xReflectMat = xReflecting(resultMat); // x축 반전
	scaleMat = scaling(resultMat, boxMat, 1.5); // 크기 변경
	leftRotateMat = rotating(resultMat, -90.0); // 90도 회전

	waitKey(2000);
	imshow("y반전 이미지", yReflectMat);
	imshow("x반전 이미지", xReflectMat);
	imshow("왼쪽 90도 회전 이미지", leftRotateMat);
	imshow("1.5배 확대 이미지", scaleMat);

	waitKey(0);

	return 0;
}