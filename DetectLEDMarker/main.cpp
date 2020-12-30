#include <opencv2/opencv.hpp>
#include <HSC/KAYACoaXpressClass.hpp>
#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <Windows.h>
#include <thread>

#ifdef _DEBUG
#define LIB_EXT "d.lib"
#else
#define LIB_EXT ".lib"
#endif


#pragma comment(lib,"KAYACoaXpressLib" LIB_EXT)
#pragma warning(disable:4996)
using namespace std;

//�O���[�o���ϐ�
/// �J�����p�����[�^
const int width = 896;
const int height = 896;
const float fps = 1000.0;
const float exposuretime = 912.0;
const int offsetx = 480;
const int offsety = 92;
double map_coeff[4], stretch_mat[4], det, distort[4];
/// �摜�Ɋւ���p�����[�^
const int cyclebuffersize = 100;
vector<cv::Mat> in_imgs;
vector<bool> processflgs;
cv::Mat zero, full;
int takepicid, in_imgs_saveid;
/// ���Ԍv���Ɋւ���p�����[�^
LARGE_INTEGER takestart, takeend, freq;
double taketime = 0;
/// �}�[�J���o�̂��߂̃p�����[�^
int detectid = 0;
cv::Mat detectimg;
uint8_t* detectimg_src;
cv::Mat detectgreen, detectblue;
const cv::Scalar greenLED_min(0, 220, 0);
const cv::Scalar greenLED_max(256, 256, 256);
const cv::Scalar blueLED_min(220, 0, 0);
const cv::Scalar blueLED_max(256, 256, 256);
vector<cv::Point> bluepts, greenpts;
double bluemass, bluemomx, bluemomy;
double greenmass[3] = { 0 }, greenmomx[3] = { 0 }, greenmomy[3] = { 0 };
double ledimpos[4][2] = { 0 }, ledidimpos[4][2] = { 0 }, greenimpos[3][2] = { 0 };
double ledcog[2] = { 0 };
double ledcamdir[4][3] = { 0 };
double lednormdir[4][3] = { 0 };
double Rm2c[3][3] = { 0 };
double Tm2c[3] = { 0 };
double* idimpos = &ledidimpos[0][0];
double* impos = &ledimpos[0][0];
double* camdir = &ledcamdir[0][0];
double* normdir = &lednormdir[0][0];
double* R = &Rm2c[0][0];
double* T = &Tm2c[0];
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



///�v���g�^�C�v�錾
void TakePicture(kayacoaxpress* cam, bool* flg);
void DetectLEDMarker();


int main() {
	bool flg = true;

	QueryPerformanceFrequency(&freq);

	kayacoaxpress cam;
	cam.connect(0);

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
	fcam = fopen("202011251943_fisheyeparam.csv", "r");
	for (size_t i = 0; i < 4; i++) { fscanf(fcam, "%lf,", &map_coeff[i]); }
	for (size_t i = 0; i < 4; i++) { fscanf(fcam, "%lf,", &stretch_mat[i]); }
	swap(stretch_mat[1], stretch_mat[2]);
	for (size_t i = 0; i < 2; i++) { fscanf(fcam, "%lf,", &distort[i]); }
	fclose(fcam);

	full = cv::Mat(cam.getParam(paramTypeCamera::paramInt::HEIGHT), cam.getParam(paramTypeCamera::paramInt::WIDTH), CV_8UC3, cv::Scalar::all(255));
	zero = cv::Mat(cam.getParam(paramTypeCamera::paramInt::HEIGHT), cam.getParam(paramTypeCamera::paramInt::WIDTH), CV_8UC3, cv::Scalar::all(0));

	//Cycle Buffer�̐���
	cout << "Set Mat Cycle Buffer..." << endl;
	for (size_t i = 0; i < cyclebuffersize; i++)
	{
		in_imgs.push_back(zero.clone());
		processflgs.push_back(false);
	}

	//�J�����N��
	cout << "Camera Start!" << endl;
	cam.start();

	//Thread�̍쐬
	thread thr1(TakePicture, &cam, &flg);


	while (flg)
	{
		DetectLEDMarker();
	}



	return 0;
}

//�摜���i�[����
void TakePicture(kayacoaxpress* cam, bool* flg) {
	cv::Mat temp = zero.clone();
	while (*flg)
	{
		QueryPerformanceCounter(&takestart);
#ifdef SAVE_IMGS_
		takepicid = in_imgs_saveid;
#endif // SAVE_IMG
#ifndef SAVE_IMGS_
		takepicid = in_imgs_saveid % cyclebuffersize;
#endif // !SAVE_IMGS
		cam->captureFrame(in_imgs[takepicid].data);
		//memcpy(in_imgs[takepicid].data, temp.data, height * width * 3);
		processflgs[takepicid] = true;
		in_imgs_saveid = (in_imgs_saveid + 1) % cyclebuffersize;
		QueryPerformanceCounter(&takeend);
		taketime = (double)(takeend.QuadPart - takestart.QuadPart) / freq.QuadPart;
		while (taketime < 0.001)
		{
			QueryPerformanceCounter(&takeend);
			taketime = (double)(takeend.QuadPart - takestart.QuadPart) / freq.QuadPart;
		}

		std::cout << "TakePicture() time: " << taketime << endl;
	}
}

void DetectLEDMarker() {
	//�摜�̊i�[
	detectid = (in_imgs_saveid - 1 + cyclebuffersize) % cyclebuffersize;
	if (processflgs[detectid])
	{
		memcpy(detectimg.data, in_imgs[detectid].data, height * width * 3);
	}
	//LED�������o�̎��́C�摜�S�̂�T������
	if (detectimg.data!=NULL && (int)detectimg.data[0]!=255)
	{
		detectimg_src = detectimg.ptr<uint8_t>(0);

		//��LED�̌��o
		cv::inRange(detectimg, blueLED_min, blueLED_max, detectblue);
		cv::findNonZero(detectblue, bluepts);
		bluemass = 0, bluemomx = 0, bluemomy = 0;
		//������Blue�����o�ł��Ȃ��������̏�����������
		for (const auto& bluept : bluepts)
		{
			bluemass += (double)detectimg_src[bluept.y * width * 3 + bluept.x * 3];
			bluemomx += (double)detectimg_src[bluept.y * width * 3 + bluept.x * 3] * bluept.x;
			bluemomy += (double)detectimg_src[bluept.y * width * 3 + bluept.x * 3] * bluept.y;
		}
		ledimpos[0][0] = bluemomx / bluemass;
		ledimpos[0][1] = bluemomy / bluemass;


		//3�̗΂�LED�̌��o
		cv::inRange(detectimg, greenLED_min, greenLED_max, detectgreen);
		//�����ŗ΂�3��LED���ނ���K�v������
		cv::findNonZero(detectgreen, greenpts);
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
		greenmass[0] = 0, greenmass[1] = 0, greenmass[2] = 0;
		greenmomx[0] = 0, greenmomx[1] = 0, greenmomx[2] = 0;
		greenmomy[0] = 0, greenmomy[1] = 0, greenmomy[2] = 0;
		green_cluster_src = green_cluster.ptr<int>(0);
		for (size_t i = 0; i < greenpts.size(); i++)
		{
			greenmass[green_cluster_src[i]] += (double)detectimg_src[greenpts[i].y * width * 3 + greenpts[i].x * 3 + 1];
			greenmomx[green_cluster_src[i]] += (double)detectimg_src[greenpts[i].y * width * 3 + greenpts[i].x * 3 + 1] * greenpts[i].x;
			greenmomy[green_cluster_src[i]] += (double)detectimg_src[greenpts[i].y * width * 3 + greenpts[i].x * 3 + 1] * greenpts[i].y;
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
			if (i==absmaxid)
			{
				ledimpos[2][0] = greenimpos[i][0];
				ledimpos[2][1] = greenimpos[i][1];
			}
			else
			{
				if (theta[i]>0)
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

	}
	//LED����x���o����Ă���Ƃ��́C�O�̌��ʂ���ROI��ݒ肷��


	//LED�̈ʒu����C4�̕����x�N�g�������߂�
	///���z�s�N�Z�����W�n�ɕϊ�
	for (size_t i = 0; i < 4; i++)
	{
		*(idimpos + i * 2 + 0) = det * ((*(impos + i * 2 + 0) -distort[0])-stretch_mat[1] * (*(impos + i * 2 + 1) - distort[1]));
		*(idimpos + i * 2 + 1) = det * (-stretch_mat[2] * (*(impos + i * 2 + 0) - distort[0]) + stretch_mat[0] * (*(impos + i * 2 + 1) - distort[1]));
	}
	///���z�s�N�Z��->�����x�N�g��
	for (size_t i = 0; i < 4; i++)
	{
		phi = hypot(*(idimpos + i * 2 + 0), *(idimpos + i * 2 + 1));
		w = map_coeff[0] + map_coeff[1] * pow(phi, 2) +
			map_coeff[2] * pow(phi, 3) + map_coeff[3] * pow(phi, 4);
		lambda = 1 / pow(pow(*(idimpos + i * 2 + 0), 2) + pow(*(idimpos + i * 2 + 1), 2) + pow(w, 2), 0.5);
		*(camdir + i * 3 + 0) = lambda * *(idimpos + i * 2 + 0);
		*(camdir + i * 3 + 1) = lambda * *(idimpos + i * 2 + 1);
		*(camdir + i * 3 + 2) = lambda * *(idimpos + i * 2 + 2);
	}

	//4�̕����x�N�g������C�Ζʂ̖@���x�N�g�������߂�
	for (size_t i = 0; i < 4; i++)
	{
		*(normdir + i * 3 + 0) = *(camdir + i * 3 + 1) * *(camdir + (i + 1) % 4 * 3 + 2) - *(camdir + i * 3 + 2) * *(camdir + (i + 1) % 4 * 3 + 1);
		*(normdir + i * 3 + 1) = *(camdir + i * 3 + 2) * *(camdir + (i + 1) % 4 * 3 + 0) - *(camdir + i * 3 + 0) * *(camdir + (i + 1) % 4 * 3 + 2);
		*(normdir + i * 3 + 2) = *(camdir + i * 3 + 0) * *(camdir + (i + 1) % 4 * 3 + 1) - *(camdir + i * 3 + 1) * *(camdir + (i + 1) % 4 * 3 + 0);
	}

	//�@���x�N�g������CLED�}�[�J�̕ӂ̕����x�N�g����2���߂�
	for (size_t i = 0; i < 2; i++)
	{
		*(R + 0 * 3 + i) = *(normdir + i * 3 + 1) * *(normdir + (i + 2) * 3 + 2) - *(normdir + i * 3 + 2) * *(normdir + (i + 2) * 3 + 1);
		*(R + 1 * 3 + i) = *(normdir + i * 3 + 2) * *(normdir + (i + 2) * 3 + 0) - *(normdir + i * 3 + 0) * *(normdir + (i + 2) * 3 + 2);
		*(R + 2 * 3 + i) = *(normdir + i * 3 + 0) * *(normdir + (i + 2) * 3 + 1) - *(normdir + i * 3 + 1) * *(normdir + (i + 2) * 3 + 0);
	}
	//�����ŁC�����x�N�g�����摜�����̌덷���悹�Ē������Ȃ��Ƃ��ɋ����ɒ�����������x�N�g�����v�Z����

	//�J����-�}�[�J�Ԃ̑��Ύp���̌v�Z(�c��̕����x�N�g�����O�ςŋ��߂�)
	*(R + 0 * 3 + 2) = *(R + 1 * 3 + 0) * *(R + 2 * 3 + 1) - *(R + 2 * 3 + 0) * *(R + 1 * 3 + 1);
	*(R + 1 * 3 + 2) = *(R + 2 * 3 + 0) * *(R + 0 * 3 + 1) - *(R + 0 * 3 + 0) * *(R + 2 * 3 + 1);
	*(R + 2 * 3 + 2) = *(R + 0 * 3 + 0) * *(R + 1 * 3 + 1) - *(R + 1 * 3 + 0) * *(R + 0 * 3 + 1);

	//���Ⴢ�f���Ƒ��Ύp����p���ăJ����-�}�[�J�Ԃ̑��Έʒu���v�Z
	for (size_t i = 0; i < 4; i++)
	{
		Asrc[i * 7 * 3 + i] = *(camdir + i * 3 + 0);
		Asrc[i * 7 * 3 + 7 + i] = *(camdir + i * 3 + 1);
		Asrc[i * 7 * 3 + 14 + i] = *(camdir + i * 3 + 2);
		Asrc[i * 7 * 3 + 4] = -1;
		Asrc[i * 7 * 3 + 12] = -1;
		Asrc[i * 7 * 3 + 20] = -1;
		bsrc[i * 3 + 0] = *(R + 0 * 3 + 0) * markerpos[i][0] + *(R + 0 * 3 + 1) * markerpos[i][1];
		bsrc[i * 3 + 1] = *(R + 1 * 3 + 0) * markerpos[i][0] + *(R + 1 * 3 + 1) * markerpos[i][1];
		bsrc[i * 3 + 2] = *(R + 2 * 3 + 0) * markerpos[i][0] + *(R + 2 * 3 + 1) * markerpos[i][1];
	}
	x = A.inv(cv::DECOMP_SVD) * b;
	*(T + 0) = xsrc[4];
	*(T + 1) = xsrc[5];
	*(T + 2) = xsrc[6];
}