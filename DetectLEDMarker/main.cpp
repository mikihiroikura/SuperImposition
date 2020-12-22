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
cv::Mat detectgreen, detectblue;
const cv::Scalar greenLED_min(0, 220, 0);
const cv::Scalar greenLED_max(256, 256, 256);
const cv::Scalar blueLED_min(220, 0, 0);
const cv::Scalar blueLED_max(256, 256, 256);
vector<cv::Point> bluepts, greenpts;
double bluemass, bluemomx, bluemomy;
double ledpos[4][2];

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
		//��LED�̌��o
		cv::inRange(detectimg, blueLED_min, blueLED_max, detectblue);
		cv::findNonZero(detectblue, bluepts);
		bluemass = 0, bluemomx = 0, bluemomy = 0;
		//������Blue�����o�ł��Ȃ��������̏�����������
		for (const auto& bluept : bluepts)
		{
			bluemass += (double)detectimg.data[bluept.y * width * 3 + bluept.x * 3];
			bluemomx += (double)detectimg.data[bluept.y * width * 3 + bluept.x * 3] * bluept.x;
			bluemomy += (double)detectimg.data[bluept.y * width * 3 + bluept.x * 3] * bluept.y;
		}
		ledpos[0][0] = bluemomx / bluemass;
		ledpos[0][1] = bluemomy / bluemass;


		//3�̗΂�LED�̌��o
		cv::inRange(detectimg, greenLED_min, greenLED_max, detectgreen);
		//�����ŗ΂�3��LED���ނ���K�v������

		
		//LED�̈ʒu����C4�̕����x�N�g�������߂�

		//4�̕����x�N�g������C�Ζʂ̖@���x�N�g�������߂�

		//�@���x�N�g������CLED�}�[�J�̕ӂ̕����x�N�g����2���߂�

		//�J����-�}�[�J�Ԃ̑��Ύp���̌v�Z

		//���Ⴢ�f���Ƒ��Ύp����p���ăJ����-�}�[�J�Ԃ̑��Έʒu���v�Z
	}
	//LED����x���o����Ă���Ƃ��́C�O�̌��ʂ���ROI��ݒ肷��
}