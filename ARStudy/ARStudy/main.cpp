#include <iostream>
#include <fstream>
#include <cmath>

#include "opencv2\highgui\highgui.hpp"
#include "opencv2\opencv.hpp"

#pragma comment(lib, "opencv_world300d.lib")

const double PI = 3.14159265;

using namespace std;
using namespace cv;

bool bLBDown = false; // ���콺 ��ư �������� üũ
bool checkDrag; // �巡�װ� �̷�������� üũ
CvRect box; // �巡�׷� �׸� �ڽ�

// �簢�� �׸���
void draw_box(IplImage* img, CvRect rect)
{
	cvRectangle(img, cvPoint(rect.x, rect.y),
		cvPoint(rect.x + rect.width, rect.y + rect.height),
		cvScalar(0xff, 0x00, 0x00));
}

// ���콺 �巡��
void on_mouse(int event, int x, int y, int flag, void* params)
{
	IplImage* image = (IplImage*)params;

	if (event == CV_EVENT_LBUTTONDOWN){ // ���� ��ư ������ ��, �ڽ� �ʱ�ȭ
		bLBDown = true;
		box = cvRect(x, y, 0, 0);
	}
	else if (event == CV_EVENT_LBUTTONUP){ // ���� ��ư �����ٰ� ���� ��, �ڽ��� ����, ���̸� �����Ѵ�.
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
	else if (event == CV_EVENT_MOUSEMOVE && bLBDown){ // �巡�� �߿��� �ڽ��� ����, ���̸� �����Ѵ�.
		box.width = x - box.x;
		box.height = y - box.y;
	}
}

// �̹��� ����
Mat copyMat(Mat source)
{
	// source�� Mat�� result�� �����ϴ� �۾�
	// opencv�� �̹� ������ �Ǿ��ִ� �۾��̴�.
	// source.copyTo(result);
	Mat result = Mat::zeros(source.size(), source.type());
	for (int i = 0; i < source.cols; i++){
		for (int j = 0; j < source.rows; j++){
			result.at<Vec3b>(j, i) = source.at<Vec3b>(j, i);
		}
	}

	return result;
}

// �ڽ��� �̹��� ����
Mat copyBoxMat(Mat source)
{
	return source(box);
}

// y�����
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

// x�����
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

// ȸ��
Mat rotating(Mat source, double degree)
{
	Mat result = copyMat(source);

	int x0 = box.x + (box.width / 2);
	int y0 = box.y + (box.height / 2);
	double cosd = cos(degree*PI / 180);
	double sind = sin(degree*PI / 180);
	
	// ������ ������ �κ����� ���� ���� 90��, ������ 90���� ����
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

// Ȯ��
Mat scaling(Mat source, Mat boxMat, double scale)
{
	Mat result = copyMat(source);
	Mat scaleBoxMat;

	// �簢�� ���� Mat�� ũ�⸦ scale�� �ø���.
	int boxWidth = (int)(boxMat.size().width * scale);
	int boxHeight = (int)(boxMat.size().height * scale);
	cv::resize(boxMat, scaleBoxMat, Size(boxWidth, boxHeight));

	// �ٿ����� �� ���� ��ġ ������ �����Ѵ�.
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

	// �̹��� �ҷ�����
    Mat gMatImage = imread("./picture/pic.jpg", 1);

	// Mat �̹����� IplImage �� �����Ѵ�.
	copy = gMatImage;
	resultImage = &copy;


	checkDrag = false;
	namedWindow("image");
	setMouseCallback("image", on_mouse, resultImage);
	cvShowImage("image", resultImage);

	//�巡�� ���
	while (!checkDrag){
		waitKey(100);
	}
	cvShowImage("image", resultImage);

	//�簢�� �߰��� ���� ����
	resultMat = cvarrToMat(resultImage);
	boxMat = copyBoxMat(resultMat);

	cout << box.x << ' ' << box.y << ' ' << box.width << ' ' << box.height << endl;

	yReflectMat = yReflecting(resultMat); // y�� ����
	xReflectMat = xReflecting(resultMat); // x�� ����
	scaleMat = scaling(resultMat, boxMat, 1.5); // ũ�� ����
	leftRotateMat = rotating(resultMat, -90.0); // 90�� ȸ��

	waitKey(2000);
	imshow("y���� �̹���", yReflectMat);
	imshow("x���� �̹���", xReflectMat);
	imshow("���� 90�� ȸ�� �̹���", leftRotateMat);
	imshow("1.5�� Ȯ�� �̹���", scaleMat);

	waitKey(0);

	return 0;
}