#include <opencv2/opencv.hpp>
#include <HSC/KAYACoaXpressClass.hpp>
#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <Windows.h>
#include <thread>
#include <vector>
#include "RS232c.h"

#ifdef _DEBUG
#define LIB_EXT "d.lib"
#else
#define LIB_EXT ".lib"
#endif


#pragma comment(lib,"KAYACoaXpressLib" LIB_EXT)
#pragma warning(disable:4996)
using namespace std;

//�O���[�o���ϐ�
/// MBED�Ɋւ���p�����[�^
RS232c mbed;
char command[256] = "";
char led = 'O';
#define READBUFFERSIZE 256
/// �J�����p�����[�^
const int width = 896;
const int height = 896;
const float fps = 1000.0;
const float exposuretime = 912.0;
const int offsetx = 512;
const int offsety = 92;
double map_coeff[4], stretch_mat[4], det, distort[4];
/// �摜�Ɋւ���p�����[�^
const int cyclebuffersize = 10;
vector<cv::Mat> in_imgs_on, in_imgs_off;
vector<bool> processflgs;
cv::Mat zero, full;
int takepicid, in_imgs_saveid;
/// ���Ԍv���Ɋւ���p�����[�^
LARGE_INTEGER takestart, takeend, freq;
LARGE_INTEGER showstart, showend;
double taketime = 0, showtime = 0;
/// �}�[�J���o�̂��߂̃p�����[�^
int detectid = 0;
vector<cv::Mat> detectimg;
cv::Mat diffimg ,diffimg_hsv;
uint8_t* diffimg_src, *detectimg_on_src;
cv::Mat detectgreen, detectblue, detectV;
const cv::Scalar greenLED_min(0, 220, 0);
const cv::Scalar greenLED_max(256, 256, 256);
const cv::Scalar blueLED_min(220, 0, 0);
const cv::Scalar blueLED_max(256, 256, 256);
const cv::Scalar HSVLED_min(0, 0, 150);
const cv::Scalar HSVLED_max(256, 256, 256);
vector<cv::Point> bluepts, greenpts, Vpts;
double bluemass, bluemomx, bluemomy;
double greenmass[3] = { 0 }, greenmomx[3] = { 0 }, greenmomy[3] = { 0 };
double ledimpos[4][2] = { 0 }, ledidimpos[4][2] = { 0 }, greenimpos[3][2] = { 0 };
double ledcog[2] = { 0 };
double ledcamdir[4][3] = { 0 };
double lednormdir[4][3] = { 0 };
double Rm2c[3][3] = { 0 };
double Tm2c[3] = { 0 };
double phi, w, lambda;
cv::Mat A = cv::Mat::zeros(12, 7, CV_64F);
cv::Mat x = cv::Mat::zeros(7, 1, CV_64F);
cv::Mat b = cv::Mat::zeros(12, 1, CV_64F);
double* Asrc = A.ptr<double>(0);
double* xsrc = x.ptr<double>(0);
double* bsrc = b.ptr<double>(0);
const double markeredge = 10;
const double markerpos[4][2] = { {markeredge, markeredge}, {-markeredge, markeredge}, {-markeredge, -markeredge}, {markeredge, -markeredge} };
cv::Mat greenptsall, green_cluster;
float* greenptsall_src;
int* green_cluster_src;
int green_cluster_num = 3;
double cross, dot, theta[3], absmax;
int absmaxid, posid, negid;
bool bluedetected = false, greendetected = false;
cv::Rect roi_blue;
vector<cv::Rect> roi_greens;
const int roi_width = 30;

#define SHOW_PROCESSING_TIME_
#define SHOW_IMGS_OPENGL_

///�v���g�^�C�v�錾
void TakePicture(kayacoaxpress* cam, bool* flg, RS232c* mbed);
void DetectLEDMarker();
void ShowAllLogs(bool* flg);


int main() {
	bool flg = true;

	QueryPerformanceFrequency(&freq);

	kayacoaxpress cam;
	cam.connect(1);

	//�p�����[�^�̐ݒ�
	cout << "Set Camera Params..." << endl;
	cam.setParam(paramTypeCamera::paramInt::WIDTH, width);
	cam.setParam(paramTypeCamera::paramInt::HEIGHT, height);
	cam.setParam(paramTypeKAYACoaXpress::paramInt::OffsetX, offsetx);
	cam.setParam(paramTypeKAYACoaXpress::paramInt::OffsetY, offsety);
	cam.setParam(paramTypeCamera::paramFloat::FPS, fps);
	cam.setParam(paramTypeKAYACoaXpress::paramFloat::ExposureTime, exposuretime);
	cam.setParam(paramTypeKAYACoaXpress::Gain::x2);
	cam.setParam(paramTypeKAYACoaXpress::CaptureType::BayerGRGrab);
	cam.parameter_all_print();

	//���[�UCalibration�̌��ʂ̌Ăяo��
	FILE* fcam;
	fcam = fopen("202012300316_fisheyeparam.csv", "r");
	for (size_t i = 0; i < 4; i++) { fscanf(fcam, "%lf,", &map_coeff[i]); }
	for (size_t i = 0; i < 4; i++) { fscanf(fcam, "%lf,", &stretch_mat[i]); }
	swap(stretch_mat[1], stretch_mat[2]);
	for (size_t i = 0; i < 2; i++) { fscanf(fcam, "%lf,", &distort[i]); }
	fclose(fcam);

	full = cv::Mat(cam.getParam(paramTypeCamera::paramInt::HEIGHT), cam.getParam(paramTypeCamera::paramInt::WIDTH), CV_8UC3, cv::Scalar::all(255));
	zero = cv::Mat(cam.getParam(paramTypeCamera::paramInt::HEIGHT), cam.getParam(paramTypeCamera::paramInt::WIDTH), CV_8UC3, cv::Scalar::all(0));

	//ROI�̐ݒ�
	roi_blue = cv::Rect(0, 0, roi_width, roi_width);
	for (size_t i = 0; i < 3; i++) roi_greens.push_back(cv::Rect(0, 0, roi_width, roi_width));

	//Cycle Buffer�̐���
	cout << "Set Mat Cycle Buffer..." << endl;
	for (size_t i = 0; i < cyclebuffersize; i++)
	{
		in_imgs_on.push_back(zero.clone());
		in_imgs_off.push_back(zero.clone());
		processflgs.push_back(false);
	}

	//LED�ʒu���o�̂��߂�Mat vector�쐬
	for (size_t i = 0; i < 2; i++)
	{
		detectimg.push_back(zero.clone());
	}
	detectimg_on_src = detectimg[0].ptr<uint8_t>(0);
	diffimg = zero.clone();
	diffimg_src = diffimg.ptr<uint8_t>(0);

	//MBED�Ƃ̐ڑ��ݒ�
	mbed.Connect("COM3", 115200, 8, NOPARITY, 0, 0, 0, 5000, 20000);
	//����J�n�̃R�}���h
	//snprintf(command, READBUFFERSIZE, "%c,\r", led);
	//mbed.Send(command);
	//memset(command, '\0', READBUFFERSIZE);

	//�J�����N��
	cout << "Camera Start!" << endl;
	cam.start();
	led = 'S';
	snprintf(command, READBUFFERSIZE, "%c,\r", led);
	mbed.Send(command);
	memset(command, '\0', READBUFFERSIZE);

	//Thread�̍쐬
	thread thr1(TakePicture, &cam, &flg, &mbed);
#ifdef SHOW_IMGS_OPENGL_
	thread thr2(ShowAllLogs, &flg);
#endif // SHOW_IMGS_OPENGL_


	while (flg)
	{
		//DetectLEDMarker();
		cout << "Detect LED marker dummy" << endl;
	}

	//�J�����̒�~�CRS232C�̐ؒf
	cam.stop();
	cam.disconnect();

	//�X���b�h�̍폜
	if (thr1.joinable()) thr1.join();
#ifdef SHOW_IMGS_OPENGL_
	if (thr2.joinable())thr2.join();
#endif // SHOW_IMGS_OPENGL_

	return 0;
}

//�摜���i�[����
void TakePicture(kayacoaxpress* cam, bool* flg, RS232c* mbed) {
	cv::Mat temp = zero.clone();
	while (*flg)
	{
		QueryPerformanceCounter(&takestart);
		takepicid = in_imgs_saveid % cyclebuffersize;
		//MBED��LED��ONOFF�R�}���h���M
		if (led == 'O') {
			cam->captureFrame(in_imgs_on[takepicid].data);
			led = 'F';
		}
		else {
			cam->captureFrame(in_imgs_off[takepicid].data);
			led = 'O';
			in_imgs_saveid = (in_imgs_saveid + 1) % cyclebuffersize;
		}
		//snprintf(command, READBUFFERSIZE, "%c,\r", led);
		//mbed->Send(command);
		//memset(command, '\0', READBUFFERSIZE);
		processflgs[takepicid] = true;
		QueryPerformanceCounter(&takeend);
		taketime = (double)(takeend.QuadPart - takestart.QuadPart) / freq.QuadPart;
		while (taketime < 0.001)
		{
			QueryPerformanceCounter(&takeend);
			taketime = (double)(takeend.QuadPart - takestart.QuadPart) / freq.QuadPart;
		}
#ifdef SHOW_PROCESSING_TIME_
		std::cout << "TakePicture() time: " << taketime << endl;
#endif // SHOW_PROCESSING_TIME_
	}
}

//�摜��OpenGL�̓_�Q�S�Ă�\��
void ShowAllLogs(bool* flg) {
	while (*flg)
	{
		QueryPerformanceCounter(&showstart);

		//OpenCV�ŉ摜�\��
		cv::imshow("img", in_imgs_on[(in_imgs_saveid - 2 + cyclebuffersize) % cyclebuffersize]);
		int key = cv::waitKey(1);
		if (key == 'q') *flg = false;

#ifdef SAVE_IMGS_
		memcpy((imglog + log_img_cnt)->data, in_imgs[(in_imgs_saveid - 2 + cyclebuffersize) % cyclebuffersize].data, height * width * 3);
		log_img_cnt++;
		if (log_img_cnt > log_img_finish_cnt) *flg = false;
#endif // SAVE_IMGS_

		QueryPerformanceCounter(&showend);
		showtime = (double)(showend.QuadPart - showstart.QuadPart) / freq.QuadPart;
		while (showtime < 0.033)
		{
			QueryPerformanceCounter(&showend);
			showtime = (double)(showend.QuadPart - showstart.QuadPart) / freq.QuadPart;
		}
#ifdef SHOW_PROCESSING_TIME_
		std::cout << "ShowAllLogs() time: " << showtime << endl;
#endif // SHOW_PROCESSING_TIME_
	}
}

void DetectLEDMarker() {
	//�摜�̊i�[
	detectid = (in_imgs_saveid - 1 + cyclebuffersize) % cyclebuffersize;
	if (processflgs[detectid])
	{
		memcpy(detectimg[0].data, in_imgs_on[detectid].data, height * width * 3);
		memcpy(detectimg[1].data, in_imgs_off[detectid].data, height * width * 3);	
	}
	//LED�������o�̎��́C�摜�S�̂�T������
	if (detectimg[0].data!=NULL && detectimg[1].data != NULL && (int)detectimg[0].data[0]!=255 && (int)detectimg[1].data[0] != 255)
	{
		//�ƗΗ������o���Ă���C�ȊO�̎�
		if (!(bluedetected&&greendetected))
		{
			//�����摜�̐���
			cv::absdiff(detectimg[0], detectimg[1], diffimg);
			cv::cvtColor(diffimg, diffimg_hsv, CV_BGR2HSV);
			//HSV�摜��V��臒l�ȏ�̍��W�����o
			cv::inRange(diffimg_hsv, HSVLED_min, HSVLED_max, detectV);
			cv::findNonZero(detectV, Vpts);

			//�Ɨ΂̉摜���W�̎擾
			bluepts.clear();
			greenpts.clear();
			for (const auto& Vpt : Vpts)
			{
				if ((double)detectimg_on_src[Vpt.y * width * 3 + Vpt.x * 3] > blueLED_min[0]) { bluepts.push_back(Vpt); }
				else if ((double)detectimg_on_src[Vpt.y * width * 3 + Vpt.x * 3 + 1] > greenLED_min[1]) { greenpts.push_back(Vpt); }
			}

			//��LED�̌��o
			bluemass = 0, bluemomx = 0, bluemomy = 0;
			//������Blue�����o�ł��Ȃ��������̏�����������
			if (bluepts.size() > 0)
			{
				for (const auto& bluept : bluepts)
				{
					bluemass += (double)diffimg_src[bluept.y * width * 3 + bluept.x * 3];
					bluemomx += (double)diffimg_src[bluept.y * width * 3 + bluept.x * 3] * bluept.x;
					bluemomy += (double)diffimg_src[bluept.y * width * 3 + bluept.x * 3] * bluept.y;
				}
				ledimpos[0][0] = bluemomx / bluemass;
				ledimpos[0][1] = bluemomy / bluemass;
				bluedetected = true;
			}

			//�΂�LED�̌��o
			if (greenpts.size() > 0)
			{
				greenptsall = cv::Mat::zeros(greenpts.size(), 2, CV_32FC1);
				green_cluster = cv::Mat::zeros(greenpts.size(), 1, CV_32SC1);
				greenptsall_src = greenptsall.ptr<float>(0);
				for (size_t i = 0; i < greenpts.size(); i++)
				{
					greenptsall_src[i * 2 + 0] = greenpts[i].x;
					greenptsall_src[i * 2 + 1] = greenpts[i].y;
				}
				//K means�@�ŗ΂�LED�̋P�_��3�̃N���X�^�ɕ���
				cv::kmeans(greenptsall, green_cluster_num, green_cluster, cvTermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1.0), 1, cv::KMEANS_PP_CENTERS);
				//������3�̃N���X�^����������3����LED�ɂȂ��Ă���Ƃ͌���Ȃ�(1��LED���狭����2�ɃN���X�^�������Ă��܂��\��������)
				//3�̃N���X�^�̏d�S�ʒu�����ꂼ��臒l�ȏ㗣��Ă��邩�ǂ������肷��


				greenmass[0] = 0, greenmass[1] = 0, greenmass[2] = 0;
				greenmomx[0] = 0, greenmomx[1] = 0, greenmomx[2] = 0;
				greenmomy[0] = 0, greenmomy[1] = 0, greenmomy[2] = 0;
				green_cluster_src = green_cluster.ptr<int>(0);
				for (size_t i = 0; i < greenpts.size(); i++)
				{
					greenmass[green_cluster_src[i]] += (double)diffimg_src[greenpts[i].y * width * 3 + greenpts[i].x * 3 + 1];
					greenmomx[green_cluster_src[i]] += (double)diffimg_src[greenpts[i].y * width * 3 + greenpts[i].x * 3 + 1] * greenpts[i].x;
					greenmomy[green_cluster_src[i]] += (double)diffimg_src[greenpts[i].y * width * 3 + greenpts[i].x * 3 + 1] * greenpts[i].y;
				}
				for (size_t i = 0; i < 3; i++)
				{
					greenimpos[i][0] = greenmomx[i] / greenmass[i];
					greenimpos[i][1] = greenmomy[i] / greenmass[i];
				}

				//�F���玞�v���ɗ�LED�̈ʒu�𓖂Ă͂߂�
				ledcog[0] = (ledimpos[0][0] + greenimpos[0][0] + greenimpos[1][0] + greenimpos[2][0]) / 4;
				ledcog[1] = (ledimpos[0][1] + greenimpos[0][1] + greenimpos[1][1] + greenimpos[2][1]) / 4;
				absmax = 0;
				for (size_t i = 0; i < 3; i++)
				{
					dot = (ledimpos[0][0] - ledcog[0]) * (greenimpos[i][0] - ledcog[0]) + (ledimpos[0][1] - ledcog[1]) * (greenimpos[i][1] - ledcog[1]);
					cross = (ledimpos[0][0] - ledcog[0]) * (greenimpos[i][1] - ledcog[1]) - (ledimpos[0][1] - ledcog[1]) * (greenimpos[i][0] - ledcog[0]);
					theta[i] = atan2(cross, dot);
					if (absmax < abs(theta[i]))
					{
						absmax = abs(theta[i]);
						absmaxid = i;
					}
				}
				for (size_t i = 0; i < 3; i++)
				{
					if (i == absmaxid)
					{
						ledimpos[2][0] = greenimpos[i][0];
						ledimpos[2][1] = greenimpos[i][1];
					}
					else
					{
						if (theta[i] > 0)
						{
							ledimpos[1][0] = greenimpos[i][0];
							ledimpos[1][1] = greenimpos[i][1];
						}
						else
						{
							ledimpos[3][0] = greenimpos[i][0];
							ledimpos[3][1] = greenimpos[i][1];
						}
					}
				}
				greendetected = true;
			}
		}

		//�ƗΗ������o���Ă���Ƃ�
		else
		{
			//��LED�̌��o�FROI��p����
			//�O�t���[���ŐFLED�����o���Ă�����ROI��ݒ肵���o����
			if (ledimpos[0][0] < roi_width / 2) {
				roi_blue.x = 0;
				roi_blue.width = roi_width / 2 + ledimpos[0][0];
			}
			else if (ledimpos[0][0] > (double)(width - roi_width / 2)) {
				roi_blue.x = ledimpos[0][0] - roi_width / 2;;
				roi_blue.width = roi_width / 2 + width - ledimpos[0][0];
			}
			else
			{
				roi_blue.x = ledimpos[0][0] - roi_width / 2;
				roi_blue.width = roi_width;
			}
			if (ledimpos[0][1] < roi_width / 2) {
				roi_blue.y = 0;
				roi_blue.height = roi_width / 2 + ledimpos[0][1];
			}
			else if (ledimpos[0][1] > (double)(height - roi_width / 2)) {
				roi_blue.y = ledimpos[0][1] - roi_width / 2;;
				roi_blue.height = roi_width / 2 + height - ledimpos[0][1];
			}
			else
			{
				roi_blue.y = ledimpos[0][1] - roi_width / 2;
				roi_blue.height = roi_width;
			}
			cv::inRange(detectimg[0](roi_blue), blueLED_min, blueLED_max, detectblue);
			cv::findNonZero(detectblue, bluepts);
			bluemass = 0, bluemomx = 0, bluemomy = 0;
			//������Blue�����o�ł��Ȃ��������̏�����������
			if (bluepts.size() > 0)
			{
				for (const auto& bluept : bluepts)
				{
					bluemass += (double)diffimg_src[(bluept.y + roi_blue.y) * width * 3 + (bluept.x + roi_blue.x) * 3];
					bluemomx += (double)diffimg_src[(bluept.y + roi_blue.y) * width * 3 + (bluept.x + roi_blue.x) * 3] * ((double)bluept.x + roi_blue.x);
					bluemomy += (double)diffimg_src[(bluept.y + roi_blue.y) * width * 3 + (bluept.x + roi_blue.x) * 3] * ((double)bluept.y + roi_blue.y);
				}
				ledimpos[0][0] = bluemomx / bluemass;
				ledimpos[0][1] = bluemomy / bluemass;
			}
			else
			{
				bluedetected = false;
			}

			//�΂�LED�̌��o�FROI��p����
			//�O�t���[���ŗ�LED��3���ׂČ��o���Ă���ꍇ�CROI��ݒ肵�Ĉʒu�����߂�
			for (size_t i = 0; i < 3; i++)
			{
				if (ledimpos[i+1][0] < roi_width / 2) {
					roi_greens[i].x = 0;
					roi_greens[i].width = roi_width / 2 + ledimpos[i + 1][0];
				}
				else if (ledimpos[i + 1][0] > (double)(width - roi_width / 2)) {
					roi_greens[i].x = ledimpos[i + 1][0] - roi_width / 2;;
					roi_greens[i].width = roi_width / 2 + width - ledimpos[i + 1][0];
				}
				else
				{
					roi_greens[i].x = ledimpos[i + 1][0] - roi_width / 2;
					roi_greens[i].width = roi_width;
				}
				if (ledimpos[i + 1][1] < roi_width / 2) {
					roi_greens[i].y = 0;
					roi_greens[i].height = roi_width / 2 + ledimpos[i + 1][1];
				}
				else if (ledimpos[i + 1][1] > (double)(height - roi_width / 2)) {
					roi_greens[i].y = ledimpos[i + 1][1] - roi_width / 2;;
					roi_greens[i].height = roi_width / 2 + height - ledimpos[i + 1][1];
				}
				else
				{
					roi_greens[i].y = ledimpos[i + 1][1] - roi_width / 2;
					roi_greens[i].height = roi_width;
				}
				cv::inRange(detectimg[0](roi_greens[i]), greenLED_min, greenLED_max, detectgreen);
				cv::findNonZero(detectgreen, greenpts);
				greenmass[i] = 0, greenmomx[i] = 0, greenmomy[i] = 0;
				if (greenpts.size() > 0)
				{
					for (const auto& greenpt : greenpts)
					{
						greenmass[i] += (double)diffimg_src[(greenpt.y + roi_greens[i].y) * width * 3 + (greenpt.x + roi_greens[i].x) * 3 + 1];
						greenmomx[i] += (double)diffimg_src[(greenpt.y + roi_greens[i].y) * width * 3 + (greenpt.x + roi_greens[i].x) * 3 + 1] * ((double)greenpt.x + roi_greens[i].x);
						greenmomy[i] += (double)diffimg_src[(greenpt.y + roi_greens[i].y) * width * 3 + (greenpt.x + roi_greens[i].x) * 3 + 1] * ((double)greenpt.y + roi_greens[i].y);
					}
					ledimpos[i + 1][0] = greenmomx[i] / greenmass[i];
					ledimpos[i + 1][1] = greenmomy[i] / greenmass[i];
				}
				else
				{
					greendetected = false;
				}
			}
		}

		

		////��LED�̌��o
		//if (!bluedetected)
		//{
		//	bluemass = 0, bluemomx = 0, bluemomy = 0;
		//	//cv::inRange(diffimg, blueLED_min, blueLED_max, detectblue);
		//	//cv::findNonZero(detectblue, bluepts);
		//	//������Blue�����o�ł��Ȃ��������̏�����������
		//	if (bluepts.size() > 0)
		//	{
		//		for (const auto& bluept : bluepts)
		//		{
		//			bluemass += (double)diffimg_src[bluept.y * width * 3 + bluept.x * 3];
		//			bluemomx += (double)diffimg_src[bluept.y * width * 3 + bluept.x * 3] * bluept.x;
		//			bluemomy += (double)diffimg_src[bluept.y * width * 3 + bluept.x * 3] * bluept.y;
		//		}
		//		ledimpos[0][0] = bluemomx / bluemass;
		//		ledimpos[0][1] = bluemomy / bluemass;
		//		bluedetected = true;
		//	}
		//	//if (Vpts.size() > 0)
		//	//{
		//	//	for (const auto& Vpt : Vpts)
		//	//	{
		//	//		if ((double)diffimg_src[Vpt.y * width * 3 + Vpt.x * 3] > blueLED_min[0])
		//	//		{
		//	//			cv::Point bluept = Vpt;
		//	//			bluemass += (double)diffimg_src[bluept.y * width * 3 + bluept.x * 3];
		//	//			bluemomx += (double)diffimg_src[bluept.y * width * 3 + bluept.x * 3] * bluept.x;
		//	//			bluemomy += (double)diffimg_src[bluept.y * width * 3 + bluept.x * 3] * bluept.y;
		//	//		}
		//	//	}
		//	//	ledimpos[0][0] = bluemomx / bluemass;
		//	//	ledimpos[0][1] = bluemomy / bluemass;
		//	//	bluedetected = true;
		//	//}
		//}
		//else
		//{//�O�t���[���ŐFLED�����o���Ă�����ROI��ݒ肵���o����
		//	if (ledimpos[0][0] < roi_width/ 2) {
		//		roi_blue.x = 0;
		//		roi_blue.width = roi_width / 2 + ledimpos[0][0];
		//	}
		//	else if(ledimpos[0][0] > (double)(width - roi_width / 2)) { 
		//		roi_blue.x = ledimpos[0][0] - roi_width / 2;;
		//		roi_blue.width = roi_width / 2  + width - ledimpos[0][0];
		//	}
		//	else
		//	{
		//		roi_blue.x = ledimpos[0][0] - roi_width / 2;
		//		roi_blue.width = roi_width;
		//	}
		//	if (ledimpos[0][1] < roi_width / 2) {
		//		roi_blue.y = 0;
		//		roi_blue.height = roi_width / 2 + ledimpos[0][1];
		//	}
		//	else if (ledimpos[0][1] > (double)(height - roi_width / 2)) {
		//		roi_blue.y = ledimpos[0][1] - roi_width / 2;;
		//		roi_blue.height = roi_width / 2 + height - ledimpos[0][1];
		//	}
		//	else
		//	{
		//		roi_blue.y = ledimpos[0][1] - roi_width / 2;
		//		roi_blue.height = roi_width;
		//	}
		//	cv::inRange(diffimg(roi_blue), blueLED_min, blueLED_max, detectblue);
		//	cv::findNonZero(detectblue, bluepts);
		//	bluemass = 0, bluemomx = 0, bluemomy = 0;
		//	//������Blue�����o�ł��Ȃ��������̏�����������
		//	if (bluepts.size() > 0)
		//	{
		//		for (const auto& bluept : bluepts)
		//		{
		//			bluemass += (double)diffimg_src[(bluept.y + roi_blue.y) * width * 3 + (bluept.x + roi_blue.x) * 3];
		//			bluemomx += (double)diffimg_src[(bluept.y + roi_blue.y) * width * 3 + (bluept.x + roi_blue.x) * 3] * ((double)bluept.x + roi_blue.x);
		//			bluemomy += (double)diffimg_src[(bluept.y + roi_blue.y) * width * 3 + (bluept.x + roi_blue.x) * 3] * ((double)bluept.y + roi_blue.y);
		//		}
		//		ledimpos[0][0] = bluemomx / bluemass;
		//		ledimpos[0][1] = bluemomy / bluemass;
		//	}
		//	else
		//	{
		//		bluedetected = false;
		//	}
		//}
		//

		////3�̗΂�LED�̌��o
		//if (!greendetected)
		//{
		//	cv::inRange(diffimg, greenLED_min, greenLED_max, detectgreen);
		//	//�����ŗ΂�3��LED���ނ���K�v������
		//	cv::findNonZero(detectgreen, greenpts);
		//	if (greenpts.size() > 0)
		//	{
		//		greenptsall = cv::Mat::zeros(greenpts.size(), 2, CV_32FC1);
		//		green_cluster = cv::Mat::zeros(greenpts.size(), 1, CV_32SC1);
		//		greenptsall_src = greenptsall.ptr<float>(0);
		//		for (size_t i = 0; i < greenpts.size(); i++)
		//		{
		//			greenptsall_src[i * 2 + 0] = greenpts[i].x;
		//			greenptsall_src[i * 2 + 1] = greenpts[i].y;
		//		}
		//		//K means�@�ŗ΂�LED�̋P�_��3�̃N���X�^�ɕ���
		//		cv::kmeans(greenptsall, green_cluster_num, green_cluster, cvTermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1.0), 1, cv::KMEANS_PP_CENTERS);
		//		//������3�̃N���X�^����������3����LED�ɂȂ��Ă���Ƃ͌���Ȃ�(1��LED���狭����2�ɃN���X�^�������Ă��܂��\��������)
		//		//3�̃N���X�^�̏d�S�ʒu�����ꂼ��臒l�ȏ㗣��Ă��邩�ǂ������肷��


		//		greenmass[0] = 0, greenmass[1] = 0, greenmass[2] = 0;
		//		greenmomx[0] = 0, greenmomx[1] = 0, greenmomx[2] = 0;
		//		greenmomy[0] = 0, greenmomy[1] = 0, greenmomy[2] = 0;
		//		green_cluster_src = green_cluster.ptr<int>(0);
		//		for (size_t i = 0; i < greenpts.size(); i++)
		//		{
		//			greenmass[green_cluster_src[i]] += (double)diffimg_src[greenpts[i].y * width * 3 + greenpts[i].x * 3 + 1];
		//			greenmomx[green_cluster_src[i]] += (double)diffimg_src[greenpts[i].y * width * 3 + greenpts[i].x * 3 + 1] * greenpts[i].x;
		//			greenmomy[green_cluster_src[i]] += (double)diffimg_src[greenpts[i].y * width * 3 + greenpts[i].x * 3 + 1] * greenpts[i].y;
		//		}
		//		for (size_t i = 0; i < 3; i++)
		//		{
		//			greenimpos[i][0] = greenmomx[i] / greenmass[i];
		//			greenimpos[i][1] = greenmomy[i] / greenmass[i];
		//		}

		//		//�F���玞�v���ɗ�LED�̈ʒu�𓖂Ă͂߂�
		//		ledcog[0] = (ledimpos[0][0] + greenimpos[0][0] + greenimpos[1][0] + greenimpos[2][0]) / 4;
		//		ledcog[1] = (ledimpos[0][1] + greenimpos[0][1] + greenimpos[1][1] + greenimpos[2][1]) / 4;
		//		absmax = 0;
		//		for (size_t i = 0; i < 3; i++)
		//		{
		//			dot = (ledimpos[0][0] - ledcog[0]) * (greenimpos[i][0] - ledcog[0]) + (ledimpos[0][1] - ledcog[1]) * (greenimpos[i][1] - ledcog[1]);
		//			cross = (ledimpos[0][0] - ledcog[0]) * (greenimpos[i][1] - ledcog[1]) - (ledimpos[0][1] - ledcog[1]) * (greenimpos[i][0] - ledcog[0]);
		//			theta[i] = atan2(cross, dot);
		//			if (absmax < abs(theta[i]))
		//			{
		//				absmax = abs(theta[i]);
		//				absmaxid = i;
		//			}
		//		}
		//		for (size_t i = 0; i < 3; i++)
		//		{
		//			if (i == absmaxid)
		//			{
		//				ledimpos[2][0] = greenimpos[i][0];
		//				ledimpos[2][1] = greenimpos[i][1];
		//			}
		//			else
		//			{
		//				if (theta[i] > 0)
		//				{
		//					ledimpos[1][0] = greenimpos[i][0];
		//					ledimpos[1][1] = greenimpos[i][1];
		//				}
		//				else
		//				{
		//					ledimpos[3][0] = greenimpos[i][0];
		//					ledimpos[3][1] = greenimpos[i][1];
		//				}
		//			}
		//		}
		//		greendetected = true;
		//	}
		//}
		//else
		//{//�O�t���[���ŗ�LED��3���ׂČ��o���Ă���ꍇ�CROI��ݒ肵�Ĉʒu�����߂�
		//	for (size_t i = 0; i < 3; i++)
		//	{
		//		roi_greens[i].x = ledimpos[i + 1][0], roi_greens[i].y = ledimpos[i + 1][1];
		//		cv::inRange(diffimg(roi_greens[i]), greenLED_min, greenLED_max, detectgreen);
		//		cv::findNonZero(detectgreen, greenpts);
		//		greenmass[i] = 0, greenmomx[i] = 0, greenmomy[i] = 0;
		//		if (greenpts.size() > 0)
		//		{
		//			for (const auto& greenpt: greenpts)
		//			{
		//				greenmass[i] += (double)diffimg_src[(greenpt.y + roi_greens[i].y) * width * 3 + (greenpt.x + roi_greens[i].x) * 3 + 1];
		//				greenmomx[i] += (double)diffimg_src[(greenpt.y + roi_greens[i].y) * width * 3 + (greenpt.x + roi_greens[i].x) * 3 + 1] * ((double)greenpt.x + roi_greens[i].x);
		//				greenmomy[i] += (double)diffimg_src[(greenpt.y + roi_greens[i].y) * width * 3 + (greenpt.x + roi_greens[i].x) * 3 + 1] * ((double)greenpt.y + roi_greens[i].y);
		//			}
		//			ledimpos[i + 1][0] = greenmomx[i] / greenmass[i];
		//			ledimpos[i + 1][1] = greenmomy[i] / greenmass[i];
		//		}
		//		else
		//		{
		//			greendetected = false;
		//		}
		//	}
		//}
	}
	else
	{
		bluedetected = false;
		greendetected = false;
	}


	//LED�̈ʒu����C4�̕����x�N�g�������߂�
	if (bluedetected && greendetected)
	{//��LED�Ɛ�LED�̗��������o���ꂽ�Ƃ��Ɉʒu�p�����X�V����
		///���z�s�N�Z�����W�n�ɕϊ�
		for (size_t i = 0; i < 4; i++)
		{
			ledidimpos[i][0] = det * ((ledimpos[i][0] - distort[0]) - stretch_mat[1] * (ledimpos[i][1] - distort[1]));
			ledidimpos[i][1] = det * (-stretch_mat[2] * (ledimpos[i][0] - distort[0]) + stretch_mat[0] * (ledimpos[i][1] - distort[1]));
		}
		///���z�s�N�Z��->�����x�N�g��
		for (size_t i = 0; i < 4; i++)
		{
			phi = hypot(ledidimpos[i][0], ledidimpos[i][1]);
			w = map_coeff[0] + map_coeff[1] * pow(phi, 2) +
				map_coeff[2] * pow(phi, 3) + map_coeff[3] * pow(phi, 4);
			lambda = 1 / pow(pow(ledidimpos[i][0], 2) + pow(ledidimpos[i][1], 2) + pow(w, 2), 0.5);
			ledcamdir[i][0] = lambda * ledidimpos[i][0];
			ledcamdir[i][1] = lambda * ledidimpos[i][1];
			ledcamdir[i][2] = lambda * w;
		}

		//4�̕����x�N�g������C�Ζʂ̖@���x�N�g�������߂�
		for (size_t i = 0; i < 4; i++)
		{
			lednormdir[i][0] = ledcamdir[i][1] * ledcamdir[(i + 1) % 4][2] - ledcamdir[i][2] * ledcamdir[(i + 1) % 4][1];
			lednormdir[i][1] = ledcamdir[i][2] * ledcamdir[(i + 1) % 4][0] - ledcamdir[i][0] * ledcamdir[(i + 1) % 4][2];
			lednormdir[i][2] = ledcamdir[i][0] * ledcamdir[(i + 1) % 4][1] - ledcamdir[i][1] * ledcamdir[(i + 1) % 4][0];
		}

		//�@���x�N�g������CLED�}�[�J�̕ӂ̕����x�N�g����2���߂�
		for (size_t i = 0; i < 2; i++)
		{
			Rm2c[0][i] = lednormdir[i][1] * lednormdir[(i + 2)][2] - lednormdir[i][2] * lednormdir[(i + 2)][1];
			Rm2c[1][i] = lednormdir[i][2] * lednormdir[(i + 2)][0] - lednormdir[i][0] * lednormdir[(i + 2)][2];
			Rm2c[2][i] = lednormdir[i][0] * lednormdir[(i + 2)][1] - lednormdir[i][1] * lednormdir[(i + 2)][0];
		}

		//�J����-�}�[�J�Ԃ̑��Ύp���̌v�Z(�c��̕����x�N�g�����O�ςŋ��߂�)
		Rm2c[0][2] = Rm2c[1][0] * Rm2c[2][1] - Rm2c[2][0] * Rm2c[1][1];
		Rm2c[1][2] = Rm2c[2][0] * Rm2c[0][1] - Rm2c[0][0] * Rm2c[2][1];
		Rm2c[2][2] = Rm2c[0][0] * Rm2c[1][1] - Rm2c[1][0] * Rm2c[0][1];

		//�����ŁC�����x�N�g�����摜�����̌덷���悹�Ē������Ȃ��Ƃ��ɋ����ɒ�����������x�N�g�����v�Z����
		Rm2c[0][1] = Rm2c[1][2] * Rm2c[2][0] - Rm2c[2][2] * Rm2c[1][0];
		Rm2c[1][1] = Rm2c[2][2] * Rm2c[0][0] - Rm2c[0][2] * Rm2c[2][0];
		Rm2c[2][1] = Rm2c[0][2] * Rm2c[1][0] - Rm2c[1][2] * Rm2c[0][0];

		//���Ⴢ�f���Ƒ��Ύp����p���ăJ����-�}�[�J�Ԃ̑��Έʒu���v�Z
		for (size_t i = 0; i < 4; i++)
		{
			Asrc[i * 7 * 3 + i] = ledcamdir[i][0];
			Asrc[i * 7 * 3 + 7 + i] = ledcamdir[i][1];
			Asrc[i * 7 * 3 + 14 + i] = ledcamdir[i][2];
			Asrc[i * 7 * 3 + 4] = -1;
			Asrc[i * 7 * 3 + 12] = -1;
			Asrc[i * 7 * 3 + 20] = -1;
			bsrc[i * 3 + 0] = Rm2c[0][0] * markerpos[i][0] + Rm2c[0][1] * markerpos[i][1];
			bsrc[i * 3 + 1] = Rm2c[1][0] * markerpos[i][0] + Rm2c[1][1] * markerpos[i][1];
			bsrc[i * 3 + 2] = Rm2c[2][0] * markerpos[i][0] + Rm2c[2][1] * markerpos[i][1];
		}
		x = A.inv(cv::DECOMP_SVD) * b;
		Tm2c[0] = xsrc[4];
		Tm2c[1] = xsrc[5];
		Tm2c[2] = xsrc[6];
	}
	
}