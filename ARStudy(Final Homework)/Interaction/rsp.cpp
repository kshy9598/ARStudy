#include "rsp.h"

// 살색 픽셀 수 체크
int checkNoise(int * noise, int noise_index, int pixel)
{
	// 프로그램 시작 후 10 번만 실행한다.
	if (noise_index > 10){
		return noise_index;
	}
	// 10번 실행 후 살색 픽셀 수의 평균값을 계산하여 살색 픽셀수를 계산해 놓는다.
	else if (noise_index == 10){
		*noise = *noise / noise_index;
		return noise_index + 1;
	}

	*noise += pixel;
	return noise_index + 1;
}

// 0이 아닌 픽셀 수 체크
int checkPixel(Mat pic)
{
	int ret = 0;
	// 0값을 넣은 픽셀 하나를 선언한다.
	Mat zpic = Mat::zeros(Size(1, 1), pic.type());

	// 0 픽셀과 비교하여 아니면 덧셈하여 정보가 있는 픽셀 수를 계산한다.
	for (int i = 0; i < pic.size().width; i++){
		for (int j = 0; j < pic.size().height; j++){
			if (pic.at<Vec3b>(j, i) != zpic.at<Vec3b>(0, 0)){
				ret++;
			}
		}
	}

	return (double)ret / (double)(pic.size().width * pic.size().height) * 100;
}

// 가위 바위 보 판단
rsp check_rsp(int pixel, int noise){
	// 기본적으로 계산해 놓은 데이터들에 의해 가위 바위 보를 판단한다.(픽셀수에 의해 결정된다)
	// 프로그램 시작시 계산한 노이즈 수를 더함으로써 노이즈에 의한 에러를 줄인다.
	if (noise + rock_low <= pixel && pixel <= noise + rock_high){
		return rock;
	}
	else if (noise + scissors_low <= pixel && pixel <= noise + scissors_high){
		return scissors;
	}
	else if (noise + paper_low <= pixel && pixel <= noise + paper_high){
		return paper;
	}

	return none;
}

// 화면 색 채우기
void fillMat(Mat * mat, int n)
{
	// 필요로 하는 곳의 데이터를 255로 만들어 빨간, 파란, 초록 색 중 하나로 변환시킨다.
	Mat pic = Mat::zeros((*mat).size(), (*mat).type());
	for (int i = 0; i < (*mat).size().width; i++){
		for (int k = 0; k < (*mat).size().height; k++){
			pic.at<Vec3b>(k, i)[n] = 255;
		}
	}

	// 색을 입힌 이미지와 원래 mat이미지를 bitwise하여 기본 이미지의 색을 바꾼다.
	bitwise_and(*mat, pic, *mat);
}

rsp image_processing(Mat camFrame, Rect box)
{
	static int noise = 0;

	Mat frame;
	frame = camFrame(box); // 웹캠 정보에서 손을 인식할 부분만 따로 떼어낸다.

	// 기존 이미지를 YCrCb 3개의 정보를 저장한 형태로 변환
	Mat rgbMat, yCbCrMat;
	cvtColor(frame, yCbCrMat, CV_RGB2YCrCb);

	// split으로 3개가 합쳐진 정보를 떼어낸다.
	// splitMatV[0] : Y, splitMatV[1] : Cr, splitMatV[2] : Cb
	vector <Mat> splitMatV;
	split(yCbCrMat, splitMatV);

	// 각각 떼어낸 이미지에서 손의 색과 비슷한 부분만을 추출한다.
	Mat crMat, cbMat;
	inRange(splitMatV[1], Scalar(77), Scalar(150), crMat);
	inRange(splitMatV[2], Scalar(133), Scalar(173), cbMat);

	// bitwise_and를 이용하여 Cr과 Cb 정보중 남아 있는 것을 and 연산하여 손의 이미지를 확정짓는다.
	Mat grayMat;
	bitwise_and(crMat, cbMat, grayMat);

	// 이미지를 BGR형태로 변환
	Mat bgrMat;
	cvtColor(grayMat, bgrMat, CV_GRAY2BGR);

	// 손의 픽셀 정보를 담고있는 bgrMat과 원래 이미지를 담고있는 frame을 and연산하여 손의 이미지만 남긴다.
	Mat handMat;
	bitwise_and(bgrMat, frame, handMat);

	// 손을 인식하여 손의 부분의 픽셀 개수를 센다.
	int pixel = checkPixel(handMat);
	cout << pixel << endl;

	// 살색 픽셀 수 체크
	static int noise_index = 0;
	noise_index = checkNoise(&noise, noise_index, pixel);

	// 픽셀 수와 노이즈 정보를 이용하여 가위 바위 보를 인식하여 그에 영상에 색을 입힌다.
	rsp temp = check_rsp(pixel, noise);
	if (temp == rock){
		cout << "rock" << endl;
		fillMat(&handMat, 2);
	}
	else if (temp == scissors){
		cout << "scissors" << endl;
		fillMat(&handMat, 1);
	}
	else if (temp == paper){
		cout << "paper" << endl;
		fillMat(&handMat, 0);
	}

	// 색까지 입혀 완성된 이미지를 원본 이미지 frame에 입혀 원본 이미지를 변환 시킨다.
	addWeighted(frame, 0.0, handMat, 1.0, 0.0, frame); 

	return temp;
}