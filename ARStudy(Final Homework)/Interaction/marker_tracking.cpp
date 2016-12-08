#include "marker_tracking.h"
#include "rsp.h"

cv::Mat gMarkerImg[MARKERNUM];	///< ��Ŀ �̹���
cv::Mat gSceneImg;	///< ī�޶�� ĸ���� �̹���
cv::Mat	gOpenGLImg[MARKERNUM];	///< OpenGL�� �������� �̹���
cv::VideoCapture gVideoCapture;	///< ī�޶� ĸ�� ��ü

cv::Ptr<cv::ORB> detector[MARKERNUM];	///< ORB Ư¡�� �����
cv::Ptr<cv::DescriptorMatcher> matcher[MARKERNUM];	///< ORB Ư¡���� ��Ī ��ü

///< ��Ŀ �� ī�޶�� ĸ���� �̹����� ORB Ư¡����(keypoints)
std::vector<cv::KeyPoint> gvMarkerKeypoints[MARKERNUM], gvSceneKeypoints[MARKERNUM];

///< ��Ŀ �� ī�޶�� ĸ���� �̹����� ORB Ư¡����(descriprtors)
cv::Mat gMarkerDescriptors[MARKERNUM], gSceneDescriptors[MARKERNUM];

cv::Mat E[MARKERNUM];	///< ��Ŀ ��ǥ�迡�� ī�޶� ��ǥ����� ��ȯ ���
cv::Mat K;	///< ī�޶� ���� �Ķ����

rsp rspdata;
Rect box;
int objColor;
int degree;

void init(void)
{
	rspdata = none;
	objColor = 0;
	degree = 0;

	// ������ ����� ���� �ν��� �簢�� ������ ������ �簢��
	box.x = 0;
	box.y = 0;
	box.height = 300;
	box.width = 300;

	///< ��Ŀ���� ī�޶���� ��ȯ ����� �ʱ�ȭ �Ѵ�.
	for (int i = 0; i < MARKERNUM; i++)
		E[i] = cv::Mat::eye(4, 4, CV_64FC1);
	K = cv::Mat::eye(3, 3, CV_64FC1);

	///< ī�޶� ���� �Ķ���� �ʱ�ȭ
	K.at<double>(0, 0) = 589.381766;
	K.at<double>(1, 1) = 570.846095;
	K.at<double>(0, 2) = 306.524379;
	K.at<double>(1, 2) = 229.923021;

	///< ��Ŀ �̹����� �д´�.
	for (int i = 0; i < MARKERNUM; i++)
		gMarkerImg[i] = cv::imread(nmarker[i], 0);

	int check = 0;
	for (check = 0; check < MARKERNUM; check++){
		if (!gMarkerImg[check].data)
			break;
	}
	///< ī�޶� �ʱ�ȭ
	if (check != MARKERNUM || !gVideoCapture.open(0)) {
		std::cerr << "�ʱ�ȭ�� ������ �� �����ϴ�." << std::endl;
		exit(-1);
	}

	///< Ư¡���� ������ ��Ī ��ü �ʱ�ȭ
	for (int i = 0; i < MARKERNUM; i++){
		detector[i] = cv::ORB::create();
		matcher[i] = cv::DescriptorMatcher::create("BruteForce-Hamming");
	}

	///< ��Ŀ ������ Ư¡���� ����
	for (int i = 0; i < MARKERNUM; i++){
		detector[i]->detect(gMarkerImg[i], gvMarkerKeypoints[i]);
		detector[i]->compute(gMarkerImg[i], gvMarkerKeypoints[i], gMarkerDescriptors[i]);
	}


	///< ��Ŀ ������ ���� ũ�� ����
	for (int k = 0; k < MARKERNUM; k++)
	{
		for (int i = 0; i < (int)gvMarkerKeypoints[k].size(); i++) {
			gvMarkerKeypoints[k][i].pt.x /= gMarkerImg[k].cols;
			gvMarkerKeypoints[k][i].pt.y /= gMarkerImg[k].rows;

			gvMarkerKeypoints[k][i].pt.x -= 0.5;
			gvMarkerKeypoints[k][i].pt.y -= 0.5;

			gvMarkerKeypoints[k][i].pt.x *= MARKER_WIDTH;
			gvMarkerKeypoints[k][i].pt.y *= MARKER_HEIGHT;
		}
	}

	///< OpenGL �ʱ�ȭ
	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
}

///< ī�޶� ���� �ĸ����Ϳ��� OpenGL ���� �Ķ���� ��ȯ
void	convertFromCaemraToOpenGLProjection(double* mGL)
{
	cv::Mat P = cv::Mat::zeros(4, 4, CV_64FC1);

	P.at<double>(0, 0) = 2 * K.at<double>(0, 0) / FRAME_WIDTH;
	P.at<double>(1, 0) = 0;
	P.at<double>(2, 0) = 0;
	P.at<double>(3, 0) = 0;

	P.at<double>(0, 1) = 0;
	P.at<double>(1, 1) = 2 * K.at<double>(1, 1) / FRAME_HEIGHT;
	P.at<double>(2, 1) = 0;
	P.at<double>(3, 1) = 0;

	P.at<double>(0, 2) = 1 - 2 * K.at<double>(0, 2) / FRAME_WIDTH;
	P.at<double>(1, 2) = -1 + (2 * K.at<double>(1, 2) + 2) / FRAME_HEIGHT;
	P.at<double>(2, 2) = (zNear + zFar) / (zNear - zFar);
	P.at<double>(3, 2) = -1;

	P.at<double>(0, 3) = 0;
	P.at<double>(1, 3) = 0;
	P.at<double>(2, 3) = 2 * zNear*zFar / (zNear - zFar);
	P.at<double>(3, 3) = 0;

	for (int ix = 0; ix < 4; ix++)
	{
		for (int iy = 0; iy < 4; iy++)
		{
			mGL[ix * 4 + iy] = P.at<double>(iy, ix);
		}
	}
}

///< ȣ��׷��Ƿκ��� ��Ŀ���� ī�޶���� ��ȯ ��� ����
bool	calculatePoseFromH(const cv::Mat& H, cv::Mat& R, cv::Mat& T)
{
	cv::Mat InvK = K.inv();
	cv::Mat InvH = InvK * H;
	cv::Mat h1 = H.col(0);
	cv::Mat h2 = H.col(1);
	cv::Mat h3 = H.col(2);

	double dbNormV1 = cv::norm(InvH.col(0));

	if (dbNormV1 != 0) {
		InvK /= dbNormV1;

		cv::Mat r1 = InvK * h1;
		cv::Mat r2 = InvK * h2;
		cv::Mat r3 = r1.cross(r2);

		T = InvK * h3;

		cv::Mat R1 = cv::Mat::zeros(3, 3, CV_64FC1);

		r1.copyTo(R1.rowRange(cv::Range::all()).col(0));
		r2.copyTo(R1.rowRange(cv::Range::all()).col(1));
		r3.copyTo(R1.rowRange(cv::Range::all()).col(2));

		cv::SVD svd(R1);

		R = svd.u * svd.vt;

		return true;
	}
	else
		return false;
}

///< ī�޶�κ��� ������ �а�, Ư¡���� ������ ��, ��Ŀ ������� ��Ī �� ȣ��׷��Ǹ� ����
///< ������ ȣ��׷��Ƿκ��� ��Ŀ���� ī�޶���� ��ȯ ��� ����
void processVideoCapture(void)
{
	cv::Mat grayImg;
	static int timer = 0;

	///< ī�޶�κ��� ����ȹ��
	gVideoCapture >> gSceneImg;

	///< Ư¡���� ������ ���Ͽ� ��鿵������ ��ȯ
	cv::cvtColor(gSceneImg, grayImg, CV_BGR2GRAY);

	///< ī�޶�κ��� ȹ���� ������ Ư¡���� ����
	for (int i = 0; i < MARKERNUM; i++){
		detector[i]->detect(grayImg, gvSceneKeypoints[i]);
		detector[i]->compute(grayImg, gvSceneKeypoints[i], gSceneDescriptors[i]);
	}

	///< ��Ŀ Ư¡������ ��Ī ����
	std::vector<std::vector<cv::DMatch>> matches[MARKERNUM];
	for (int i = 0; i < MARKERNUM; i++){
		matcher[i]->knnMatch(gMarkerDescriptors[i], gSceneDescriptors[i], matches[i], 2);
	}

	std::vector<cv::DMatch> good_matches[MARKERNUM];
	///< ��Ŀ Ư¡������ ����� �������� �ִ� ��쿡....
	for (int k = 0; k < MARKERNUM; k++){
		for (int i = 0; i < (int)matches[k].size(); i++) {
			if (matches[k][i][0].distance < 0.9 * matches[k][i][1].distance) {
				good_matches[k].push_back(matches[k][i][0]);
			}
		}

		if (good_matches[k].size() > 10) {
			std::vector<cv::Point2f> vMarkerPts;
			std::vector<cv::Point2f> vScenePts;

			///< ȣ��׷��� ����
			for (int i = 0; i < (int)good_matches[k].size(); i++) {
				vMarkerPts.push_back(gvMarkerKeypoints[k][matches[k][i][0].queryIdx].pt);
				vScenePts.push_back(gvSceneKeypoints[k][matches[k][i][0].trainIdx].pt);
			}

			cv::Mat H = cv::findHomography(vMarkerPts, vScenePts, CV_RANSAC);

			std::vector<cv::Point2f> obj_corners(4);
			obj_corners[0] = cv::Point(-MARKER_WIDTH / 2, -MARKER_HEIGHT / 2);
			obj_corners[1] = cv::Point(MARKER_WIDTH / 2, -MARKER_HEIGHT / 2);
			obj_corners[2] = cv::Point(MARKER_WIDTH / 2, MARKER_HEIGHT / 2);
			obj_corners[3] = cv::Point(-MARKER_WIDTH / 2, MARKER_HEIGHT / 2);

			std::vector<cv::Point2f> scene_corners(4);

			cv::perspectiveTransform(obj_corners, scene_corners, H);

			cv::line(gSceneImg, scene_corners[0], scene_corners[1], cv::Scalar(0, 255, 0), 2);
			cv::line(gSceneImg, scene_corners[1], scene_corners[2], cv::Scalar(0, 255, 0), 2);
			cv::line(gSceneImg, scene_corners[2], scene_corners[3], cv::Scalar(0, 255, 0), 2);
			cv::line(gSceneImg, scene_corners[3], scene_corners[0], cv::Scalar(0, 255, 0), 2);

			cv::Mat R, T;

			///< ȣ��׷��Ƿκ��� ��Ŀ���� ī�޶� ��ǥ���� ��ȯ ��� ����
			if (calculatePoseFromH(H, R, T)) {
				R.copyTo(E[k].rowRange(0, 3).colRange(0, 3));
				T.copyTo(E[k].rowRange(0, 3).col(3));

				static double changeCoordArray[4][4] = { { 1, 0, 0, 0 }, { 0, -1, 0, 0 }, { 0, 0, -1, 0 }, { 0, 0, 0, 1 } };
				static cv::Mat changeCoord(4, 4, CV_64FC1, changeCoordArray);

				E[k] = changeCoord * E[k];
			}
		}
	}

	timer++;
	if (timer == 11){
		rspdata = image_processing(gSceneImg, box);

		if (rspdata == scissors){
			objColor += 1;
			objColor %= 3;
		}
		timer = 0;
	}
	else{
		image_processing(gSceneImg, box);
	}

	if (gSceneImg.data)
		cv::flip(gSceneImg, gOpenGLImg[0], 0);

	glutPostRedisplay();
}

void display(void)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	if (rspdata == rock)
		degree = (degree + 1);

	for (int k = 0; k < MARKERNUM; k++){
		///< ��� ���� ������
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();

		glDrawPixels(gOpenGLImg[k].cols, gOpenGLImg[k].rows, GL_BGR_EXT, GL_UNSIGNED_BYTE, (void *)gOpenGLImg[k].data);

		///< ��Ŀ�κ��� ī�޶���� ��ȯ����� ���� ��Ŀ��ǥ����� ��ȯ
		glMatrixMode(GL_MODELVIEW);

		cv::Mat E1;
		E1 = E[k].t();

		glMultMatrixd((double *)E1.data);
		glLineWidth(2.0f);
		glBegin(GL_LINES);
		glColor3f(1.0f, 0.0f, 0.0f); glVertex3d(0.0, 0.0, 0.0); glVertex3d(10.0, 0.0, 0.0);
		glColor3f(0.0f, 1.0f, 0.0f); glVertex3d(0.0, 0.0, 0.0); glVertex3d(0.0, 10.0, 0.0);
		glColor3f(0.0f, 0.0f, 1.0f); glVertex3d(0.0, 0.0, 0.0); glVertex3d(0.0, 0.0, 10.0);
		glEnd();

		glLineWidth(1.0f);

		if (k == 0){
			glRotated(degree * 30, 0.0, 0.0, 1.0);
			glTranslated(3 * sin(degree), 3 * cos(degree), 0.0);
		}
		if (k == 1){
			glRotated(-degree * 30, 0.0, 0.0, 1.0);
			glTranslated(3 * sin(-degree), 3 * cos(-degree), 0.0);
		}

		///< ��Ŀ ��ǥ���� �߽ɿ��� ��ü ������
		if (rspdata == paper){
			if (k == 0){
				printWireCube();
			}
			else{
				printWireTeapot();
			}
		}
		else{
			if (k == 0){
				printSolidCube();
			}
			else{
				printSolidTeapot();
			}
		}
	}

	glutSwapBuffers();
}

void idle(void)
{
	processVideoCapture();

	glutPostRedisplay();
}

void reshape(int w, int h)
{
	double P[16] = { 0 };

	convertFromCaemraToOpenGLProjection(P);

	glViewport(0, 0, w, h);
	glMatrixMode(GL_PROJECTION);
	glLoadMatrixd(P);
	glMatrixMode(GL_MODELVIEW);
}

void printSolidTeapot()
{
	if (objColor == 0)
		glColor3f(0.0, 0.0, 1.0f);
	if (objColor == 1)
		glColor3f(1.0f, 0.0, 0.0);
	if (objColor == 2)
		glColor3f(0.0, 1.0f, 0.0);
	glutSolidTeapot(objectSize);
}

void printWireTeapot()
{
	if (objColor == 0)
		glColor3f(0.0, 0.0, 1.0f);
	if (objColor == 1)
		glColor3f(1.0f, 0.0, 0.0);
	if (objColor == 2)
		glColor3f(0.0, 1.0f, 0.0);
	glutWireTeapot(objectSize);
}

void printSolidCube()
{
	if (objColor == 0)
		glColor3f(1.0f, 0.0, 0.0);
	if (objColor == 1)
		glColor3f(0.0, 1.0f, 0.0);
	if (objColor == 2)
		glColor3f(0.0, 0.0, 1.0f);
	glutSolidCube(objectSize);
}

void printWireCube()
{
	if (objColor == 0)
		glColor3f(1.0f, 0.0, 0.0);
	if (objColor == 1)
		glColor3f(0.0, 1.0f, 0.0);
	if (objColor == 2)
		glColor3f(0.0, 0.0, 1.0f);
	glutWireCube(objectSize);
}