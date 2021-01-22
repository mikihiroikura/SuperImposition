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

//グローバル変数
/// MBEDに関するパラメータ
RS232c mbed;
char command[256] = "";
char led = 'O';
#define READBUFFERSIZE 256
/// カメラパラメータ
const int width = 896;
const int height = 896;
const float fps = 1000.0;
const float exposuretime = 912.0;
const int offsetx = 512;
const int offsety = 92;
double map_coeff[4], stretch_mat[4], det, distort[4];
/// 画像に関するパラメータ
const int cyclebuffersize = 10;
vector<cv::Mat> in_imgs_on, in_imgs_off;
vector<bool> processflgs;
cv::Mat zero, full;
int takepicid, in_imgs_saveid;
/// 時間計測に関するパラメータ
LARGE_INTEGER takestart, takeend, freq;
LARGE_INTEGER showstart, showend;
double taketime = 0, showtime = 0;
/// マーカ検出のためのパラメータ
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

///プロトタイプ宣言
void TakePicture(kayacoaxpress* cam, bool* flg, RS232c* mbed);
void DetectLEDMarker();
void ShowAllLogs(bool* flg);


int main() {
	bool flg = true;

	QueryPerformanceFrequency(&freq);

	kayacoaxpress cam;
	cam.connect(1);

	//パラメータの設定
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

	//レーザCalibrationの結果の呼び出し
	FILE* fcam;
	fcam = fopen("202012300316_fisheyeparam.csv", "r");
	for (size_t i = 0; i < 4; i++) { fscanf(fcam, "%lf,", &map_coeff[i]); }
	for (size_t i = 0; i < 4; i++) { fscanf(fcam, "%lf,", &stretch_mat[i]); }
	swap(stretch_mat[1], stretch_mat[2]);
	for (size_t i = 0; i < 2; i++) { fscanf(fcam, "%lf,", &distort[i]); }
	fclose(fcam);

	full = cv::Mat(cam.getParam(paramTypeCamera::paramInt::HEIGHT), cam.getParam(paramTypeCamera::paramInt::WIDTH), CV_8UC3, cv::Scalar::all(255));
	zero = cv::Mat(cam.getParam(paramTypeCamera::paramInt::HEIGHT), cam.getParam(paramTypeCamera::paramInt::WIDTH), CV_8UC3, cv::Scalar::all(0));

	//ROIの設定
	roi_blue = cv::Rect(0, 0, roi_width, roi_width);
	for (size_t i = 0; i < 3; i++) roi_greens.push_back(cv::Rect(0, 0, roi_width, roi_width));

	//Cycle Bufferの生成
	cout << "Set Mat Cycle Buffer..." << endl;
	for (size_t i = 0; i < cyclebuffersize; i++)
	{
		in_imgs_on.push_back(zero.clone());
		in_imgs_off.push_back(zero.clone());
		processflgs.push_back(false);
	}

	//LED位置検出のためのMat vector作成
	for (size_t i = 0; i < 2; i++)
	{
		detectimg.push_back(zero.clone());
	}
	detectimg_on_src = detectimg[0].ptr<uint8_t>(0);
	diffimg = zero.clone();
	diffimg_src = diffimg.ptr<uint8_t>(0);

	//MBEDとの接続設定
	mbed.Connect("COM3", 115200, 8, NOPARITY, 0, 0, 0, 5000, 20000);
	//動作開始のコマンド
	//snprintf(command, READBUFFERSIZE, "%c,\r", led);
	//mbed.Send(command);
	//memset(command, '\0', READBUFFERSIZE);

	//カメラ起動
	cout << "Camera Start!" << endl;
	cam.start();
	led = 'S';
	snprintf(command, READBUFFERSIZE, "%c,\r", led);
	mbed.Send(command);
	memset(command, '\0', READBUFFERSIZE);

	//Threadの作成
	thread thr1(TakePicture, &cam, &flg, &mbed);
#ifdef SHOW_IMGS_OPENGL_
	thread thr2(ShowAllLogs, &flg);
#endif // SHOW_IMGS_OPENGL_


	while (flg)
	{
		//DetectLEDMarker();
		cout << "Detect LED marker dummy" << endl;
	}

	//カメラの停止，RS232Cの切断
	cam.stop();
	cam.disconnect();

	//スレッドの削除
	if (thr1.joinable()) thr1.join();
#ifdef SHOW_IMGS_OPENGL_
	if (thr2.joinable())thr2.join();
#endif // SHOW_IMGS_OPENGL_

	return 0;
}

//画像を格納する
void TakePicture(kayacoaxpress* cam, bool* flg, RS232c* mbed) {
	cv::Mat temp = zero.clone();
	while (*flg)
	{
		QueryPerformanceCounter(&takestart);
		takepicid = in_imgs_saveid % cyclebuffersize;
		//MBEDにLEDのONOFFコマンド送信
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

//画像とOpenGLの点群全てを表示
void ShowAllLogs(bool* flg) {
	while (*flg)
	{
		QueryPerformanceCounter(&showstart);

		//OpenCVで画像表示
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
	//画像の格納
	detectid = (in_imgs_saveid - 1 + cyclebuffersize) % cyclebuffersize;
	if (processflgs[detectid])
	{
		memcpy(detectimg[0].data, in_imgs_on[detectid].data, height * width * 3);
		memcpy(detectimg[1].data, in_imgs_off[detectid].data, height * width * 3);	
	}
	//LEDが未検出の時は，画像全体を探索する
	if (detectimg[0].data!=NULL && detectimg[1].data != NULL && (int)detectimg[0].data[0]!=255 && (int)detectimg[1].data[0] != 255)
	{
		//青と緑両方検出していろ，以外の時
		if (!(bluedetected&&greendetected))
		{
			//差分画像の生成
			cv::absdiff(detectimg[0], detectimg[1], diffimg);
			cv::cvtColor(diffimg, diffimg_hsv, CV_BGR2HSV);
			//HSV画像でVが閾値以上の座標を検出
			cv::inRange(diffimg_hsv, HSVLED_min, HSVLED_max, detectV);
			cv::findNonZero(detectV, Vpts);

			//青と緑の画像座標の取得
			bluepts.clear();
			greenpts.clear();
			for (const auto& Vpt : Vpts)
			{
				if ((double)detectimg_on_src[Vpt.y * width * 3 + Vpt.x * 3] > blueLED_min[0]) { bluepts.push_back(Vpt); }
				else if ((double)detectimg_on_src[Vpt.y * width * 3 + Vpt.x * 3 + 1] > greenLED_min[1]) { greenpts.push_back(Vpt); }
			}

			//青のLEDの検出
			bluemass = 0, bluemomx = 0, bluemomy = 0;
			//ここにBlueが検出できなかった時の処理を加える
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

			//緑のLEDの検出
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
				//K means法で緑のLEDの輝点を3つのクラスタに分類
				cv::kmeans(greenptsall, green_cluster_num, green_cluster, cvTermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1.0), 1, cv::KMEANS_PP_CENTERS);
				//ここで3つのクラスタがしっかり3つ分のLEDになっているとは限らない(1つのLEDから強引に2つにクラスタ分けしてしまう可能性がある)
				//3つのクラスタの重心位置がそれぞれ閾値以上離れているかどうか判定する


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

				//青色から時計回りに緑LEDの位置を当てはめる
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

		//青と緑両方検出しているとき
		else
		{
			//青のLEDの検出：ROIを用いる
			//前フレームで青色LEDを検出していたらROIを設定し検出する
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
			//ここにBlueが検出できなかった時の処理を加える
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

			//緑のLEDの検出：ROIを用いる
			//前フレームで緑LEDを3つすべて検出している場合，ROIを設定して位置を求める
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

		

		////青のLEDの検出
		//if (!bluedetected)
		//{
		//	bluemass = 0, bluemomx = 0, bluemomy = 0;
		//	//cv::inRange(diffimg, blueLED_min, blueLED_max, detectblue);
		//	//cv::findNonZero(detectblue, bluepts);
		//	//ここにBlueが検出できなかった時の処理を加える
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
		//{//前フレームで青色LEDを検出していたらROIを設定し検出する
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
		//	//ここにBlueが検出できなかった時の処理を加える
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

		////3つの緑のLEDの検出
		//if (!greendetected)
		//{
		//	cv::inRange(diffimg, greenLED_min, greenLED_max, detectgreen);
		//	//ここで緑の3つのLED分類する必要がある
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
		//		//K means法で緑のLEDの輝点を3つのクラスタに分類
		//		cv::kmeans(greenptsall, green_cluster_num, green_cluster, cvTermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1.0), 1, cv::KMEANS_PP_CENTERS);
		//		//ここで3つのクラスタがしっかり3つ分のLEDになっているとは限らない(1つのLEDから強引に2つにクラスタ分けしてしまう可能性がある)
		//		//3つのクラスタの重心位置がそれぞれ閾値以上離れているかどうか判定する


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

		//		//青色から時計回りに緑LEDの位置を当てはめる
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
		//{//前フレームで緑LEDを3つすべて検出している場合，ROIを設定して位置を求める
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


	//LEDの位置から，4つの方向ベクトルを求める
	if (bluedetected && greendetected)
	{//緑LEDと青LEDの両方が検出されたときに位置姿勢を更新する
		///理想ピクセル座標系に変換
		for (size_t i = 0; i < 4; i++)
		{
			ledidimpos[i][0] = det * ((ledimpos[i][0] - distort[0]) - stretch_mat[1] * (ledimpos[i][1] - distort[1]));
			ledidimpos[i][1] = det * (-stretch_mat[2] * (ledimpos[i][0] - distort[0]) + stretch_mat[0] * (ledimpos[i][1] - distort[1]));
		}
		///理想ピクセル->方向ベクトル
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

		//4つの方向ベクトルから，斜面の法線ベクトルを求める
		for (size_t i = 0; i < 4; i++)
		{
			lednormdir[i][0] = ledcamdir[i][1] * ledcamdir[(i + 1) % 4][2] - ledcamdir[i][2] * ledcamdir[(i + 1) % 4][1];
			lednormdir[i][1] = ledcamdir[i][2] * ledcamdir[(i + 1) % 4][0] - ledcamdir[i][0] * ledcamdir[(i + 1) % 4][2];
			lednormdir[i][2] = ledcamdir[i][0] * ledcamdir[(i + 1) % 4][1] - ledcamdir[i][1] * ledcamdir[(i + 1) % 4][0];
		}

		//法線ベクトルから，LEDマーカの辺の方向ベクトルを2つ求める
		for (size_t i = 0; i < 2; i++)
		{
			Rm2c[0][i] = lednormdir[i][1] * lednormdir[(i + 2)][2] - lednormdir[i][2] * lednormdir[(i + 2)][1];
			Rm2c[1][i] = lednormdir[i][2] * lednormdir[(i + 2)][0] - lednormdir[i][0] * lednormdir[(i + 2)][2];
			Rm2c[2][i] = lednormdir[i][0] * lednormdir[(i + 2)][1] - lednormdir[i][1] * lednormdir[(i + 2)][0];
		}

		//カメラ-マーカ間の相対姿勢の計算(残りの方向ベクトルを外積で求める)
		Rm2c[0][2] = Rm2c[1][0] * Rm2c[2][1] - Rm2c[2][0] * Rm2c[1][1];
		Rm2c[1][2] = Rm2c[2][0] * Rm2c[0][1] - Rm2c[0][0] * Rm2c[2][1];
		Rm2c[2][2] = Rm2c[0][0] * Rm2c[1][1] - Rm2c[1][0] * Rm2c[0][1];

		//ここで，方向ベクトルが画像処理の誤差を乗せて直交しないときに強引に直交する方向ベクトルを計算する
		Rm2c[0][1] = Rm2c[1][2] * Rm2c[2][0] - Rm2c[2][2] * Rm2c[1][0];
		Rm2c[1][1] = Rm2c[2][2] * Rm2c[0][0] - Rm2c[0][2] * Rm2c[2][0];
		Rm2c[2][1] = Rm2c[0][2] * Rm2c[1][0] - Rm2c[1][2] * Rm2c[0][0];

		//魚眼モデルと相対姿勢を用いてカメラ-マーカ間の相対位置を計算
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