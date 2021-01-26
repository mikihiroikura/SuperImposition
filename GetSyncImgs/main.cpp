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

/// カメラパラメータ
const int width = 896;
const int height = 896;
const float fps = 1000.0;
const float exposuretime = 912.0;
const int offsetx = 512;
const int offsety = 92;
/// 画像に関するパラメータ
const int cyclebuffersize = 10;
vector<cv::Mat> in_imgs_on, in_imgs_off, in_imgs;
vector<bool> processflgs;
cv::Mat zero, full;
int takepicid, in_imgs_saveid;
/// 時間計測に関するパラメータ
LARGE_INTEGER takestart, takeend, freq;
LARGE_INTEGER showstart, showend;
double taketime = 0, showtime = 0;

#define SHOW_PROCESSING_TIME_

//プロトタイプ宣言
void TakePicture(kayacoaxpress* cam, bool* flg);
void ShowAllLogs(bool* flg);

int main() {
	bool flg = true;

	QueryPerformanceFrequency(&freq);

	kayacoaxpress cam;
	cam.connect(1);

	//パラメータの設定
	cout << "Set Camera Params..." << endl;
	cam.setParam(paramTypeKAYACoaXpress::AcquisitionMode::TriggerMode, 1);
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

	//Cycle Bufferの生成
	cout << "Set Mat Cycle Buffer..." << endl;
	for (size_t i = 0; i < cyclebuffersize; i++)
	{
		in_imgs_on.push_back(zero.clone());
		in_imgs_off.push_back(zero.clone());
		in_imgs.push_back(zero.clone());
		processflgs.push_back(false);
	}

	//カメラ起動
	cout << "Camera Start!" << endl;
	cam.start();

	thread thr2(ShowAllLogs, &flg);

	cv::Mat temp = zero.clone();
	while (flg)
	{
		QueryPerformanceCounter(&takestart);
		takepicid = in_imgs_saveid % cyclebuffersize;
		//MBEDにLEDのONOFFコマンド送信
		/*if (takepicid % 2 == 0) {
			cam.captureFrame(in_imgs_on[takepicid].data);
		}
		else {
			cam.captureFrame(in_imgs_off[takepicid].data);
		}*/
		cam.captureFrame(in_imgs[takepicid].data);
		if (takepicid % 2 == 0) {
			memcpy(in_imgs_on[takepicid / 2].data, in_imgs[takepicid].data, height * width * 3);
		}
		else
		{
			memcpy(in_imgs_off[takepicid / 2].data, in_imgs[takepicid].data, height * width * 3);
		}
		in_imgs_saveid = (in_imgs_saveid + 1) % cyclebuffersize;
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

	//カメラの停止，RS232Cの切断
	cam.stop();
	cam.disconnect();

	if (thr2.joinable())thr2.join();

	return 0;
}

////画像を格納する
//void TakePicture(kayacoaxpress* cam, bool* flg) {
//	cv::Mat temp = zero.clone();
//	while (*flg)
//	{
//		QueryPerformanceCounter(&takestart);
//		takepicid = in_imgs_saveid % cyclebuffersize;
//		//MBEDにLEDのONOFFコマンド送信
//		if (takepicid%2==0) {
//			cam->captureFrame(in_imgs_on[takepicid].data);
//		}
//		else {
//			cam->captureFrame(in_imgs_off[takepicid].data);
//			in_imgs_saveid = (in_imgs_saveid + 1) % cyclebuffersize;
//		}
//		processflgs[takepicid] = true;
//		QueryPerformanceCounter(&takeend);
//		taketime = (double)(takeend.QuadPart - takestart.QuadPart) / freq.QuadPart;
//		while (taketime < 1.0)
//		{
//			QueryPerformanceCounter(&takeend);
//			taketime = (double)(takeend.QuadPart - takestart.QuadPart) / freq.QuadPart;
//		}
//#ifdef SHOW_PROCESSING_TIME_
//		std::cout << "TakePicture() time: " << taketime << endl;
//#endif // SHOW_PROCESSING_TIME_
//	}
//}

//画像とOpenGLの点群全てを表示
void ShowAllLogs(bool* flg) {
	while (*flg)
	{
		QueryPerformanceCounter(&showstart);

		//OpenCVで画像表示
		cv::imshow("img", in_imgs[(in_imgs_saveid - 2 + cyclebuffersize) % cyclebuffersize]);
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