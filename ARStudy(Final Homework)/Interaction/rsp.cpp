#include "rsp.h"

// ��� �ȼ� �� üũ
int checkNoise(int * noise, int noise_index, int pixel)
{
	// ���α׷� ���� �� 10 ���� �����Ѵ�.
	if (noise_index > 10){
		return noise_index;
	}
	// 10�� ���� �� ��� �ȼ� ���� ��հ��� ����Ͽ� ��� �ȼ����� ����� ���´�.
	else if (noise_index == 10){
		*noise = *noise / noise_index;
		return noise_index + 1;
	}

	*noise += pixel;
	return noise_index + 1;
}

// 0�� �ƴ� �ȼ� �� üũ
int checkPixel(Mat pic)
{
	int ret = 0;
	// 0���� ���� �ȼ� �ϳ��� �����Ѵ�.
	Mat zpic = Mat::zeros(Size(1, 1), pic.type());

	// 0 �ȼ��� ���Ͽ� �ƴϸ� �����Ͽ� ������ �ִ� �ȼ� ���� ����Ѵ�.
	for (int i = 0; i < pic.size().width; i++){
		for (int j = 0; j < pic.size().height; j++){
			if (pic.at<Vec3b>(j, i) != zpic.at<Vec3b>(0, 0)){
				ret++;
			}
		}
	}

	return (double)ret / (double)(pic.size().width * pic.size().height) * 100;
}

// ���� ���� �� �Ǵ�
rsp check_rsp(int pixel, int noise){
	// �⺻������ ����� ���� �����͵鿡 ���� ���� ���� ���� �Ǵ��Ѵ�.(�ȼ����� ���� �����ȴ�)
	// ���α׷� ���۽� ����� ������ ���� �������ν� ����� ���� ������ ���δ�.
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

// ȭ�� �� ä���
void fillMat(Mat * mat, int n)
{
	// �ʿ�� �ϴ� ���� �����͸� 255�� ����� ����, �Ķ�, �ʷ� �� �� �ϳ��� ��ȯ��Ų��.
	Mat pic = Mat::zeros((*mat).size(), (*mat).type());
	for (int i = 0; i < (*mat).size().width; i++){
		for (int k = 0; k < (*mat).size().height; k++){
			pic.at<Vec3b>(k, i)[n] = 255;
		}
	}

	// ���� ���� �̹����� ���� mat�̹����� bitwise�Ͽ� �⺻ �̹����� ���� �ٲ۴�.
	bitwise_and(*mat, pic, *mat);
}

rsp image_processing(Mat camFrame, Rect box)
{
	static int noise = 0;

	Mat frame;
	frame = camFrame(box); // ��ķ �������� ���� �ν��� �κи� ���� �����.

	// ���� �̹����� YCrCb 3���� ������ ������ ���·� ��ȯ
	Mat rgbMat, yCbCrMat;
	cvtColor(frame, yCbCrMat, CV_RGB2YCrCb);

	// split���� 3���� ������ ������ �����.
	// splitMatV[0] : Y, splitMatV[1] : Cr, splitMatV[2] : Cb
	vector <Mat> splitMatV;
	split(yCbCrMat, splitMatV);

	// ���� ��� �̹������� ���� ���� ����� �κи��� �����Ѵ�.
	Mat crMat, cbMat;
	inRange(splitMatV[1], Scalar(77), Scalar(150), crMat);
	inRange(splitMatV[2], Scalar(133), Scalar(173), cbMat);

	// bitwise_and�� �̿��Ͽ� Cr�� Cb ������ ���� �ִ� ���� and �����Ͽ� ���� �̹����� Ȯ�����´�.
	Mat grayMat;
	bitwise_and(crMat, cbMat, grayMat);

	// �̹����� BGR���·� ��ȯ
	Mat bgrMat;
	cvtColor(grayMat, bgrMat, CV_GRAY2BGR);

	// ���� �ȼ� ������ ����ִ� bgrMat�� ���� �̹����� ����ִ� frame�� and�����Ͽ� ���� �̹����� �����.
	Mat handMat;
	bitwise_and(bgrMat, frame, handMat);

	// ���� �ν��Ͽ� ���� �κ��� �ȼ� ������ ����.
	int pixel = checkPixel(handMat);
	cout << pixel << endl;

	// ��� �ȼ� �� üũ
	static int noise_index = 0;
	noise_index = checkNoise(&noise, noise_index, pixel);

	// �ȼ� ���� ������ ������ �̿��Ͽ� ���� ���� ���� �ν��Ͽ� �׿� ���� ���� ������.
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

	// ������ ���� �ϼ��� �̹����� ���� �̹��� frame�� ���� ���� �̹����� ��ȯ ��Ų��.
	addWeighted(frame, 0.0, handMat, 1.0, 0.0, frame); 

	return temp;
}