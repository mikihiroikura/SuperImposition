#define _USE_MATH_DEFINES

#include <opencv2/opencv.hpp>
#include <HSC/KAYACoaXpressClass.hpp>
#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <Windows.h>
#include <thread>
#include <vector>
#include <cmath>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <cmath>
#include <iostream>
#include <Windows.h>
#include <thread>
#include <vector>
#include "graphics.h"
#include <cstdio>
#include <iostream>
#include <direct.h>
#include <sys/stat.h>
#include "RS232c.h"

#ifdef _DEBUG
#define LIB_EXT "d.lib"
#else
#define LIB_EXT ".lib"
#endif


#pragma comment(lib,"KAYACoaXpressLib" LIB_EXT)
#pragma warning(disable:4996)
using namespace std;

//スレッドの処理時間
double takepic_time = 0.002;
double showgl_time = 0.01667;
double showhsc_time = 0.0333;
double calcpose_time = 0.002;


//カメラパラメータ
const int width = 896;
const int height = 896;
const float fps = 1000.0;
const float exposuretime = 912.0;
const int offsetx = 512;
const int offsety = 92;
double map_coeff[4], stretch_mat[4], det, distort[4];

/// 画像に関するパラメータ
const int ringbuffersize = 10;
vector<cv::Mat> in_imgs_on, in_imgs_off, in_imgs;
vector<bool> processflgs;
cv::Mat zero, full, zeromulti, show_img, logsaveimg;
int takepicid, in_imgs_saveid, log_img_saveid;
const int multicnt = 2;
uint8_t* in_img_multi_src, * detectimg_multi_src;

//Realsenseに関するパラメータ
vector<realsense> rs_devices;
rs2::context context;
const int ring_size_realsense = 5;
int getpc_id = 0;
float* texcoords_src;
int update_ringid;
struct PointCloud
{
	const rs2::vertex* pc_buffer;
	const rs2::texture_coordinate* texcoords;
	float* pc_ringbuffer;
	float* texcoords_ringbuffer;
	rs2::frame colorframe_buffer[ring_size_realsense];
	int pc_ringid = 0;
};

//出力に関するパラメータ
float* gl_pc_src[realsense_cnt];
float* gl_texcoord_src[realsense_cnt];
rs2::frame* gl_tex_src[realsense_cnt];

//時間に関する変数
LARGE_INTEGER start, stop, freq;
LARGE_INTEGER takestart, takeend;
LARGE_INTEGER savehscstart, savehscend;
LARGE_INTEGER hscstart, hscend;
LARGE_INTEGER glstart, glend;
LARGE_INTEGER logstart, logend, glrslogend, hsclogend;
LARGE_INTEGER detectstart, detectstartdebug, detectend;
double taketime = 0, hsctime = 0, logtime = 0, gltime = 0, glrslogtime = 0, hsclogtime = 0, savehsctime = 0;
double timer = 0, gltimer = 0;
double detecttimea = 0, detecttimeb = 0, detecttimec = 0, detecttimed = 0, detecttimee = 0, detecttimef = 0, detecttime = 0;

//マーカ検出に関するパラメータ
int detectid = 0;
vector<cv::Mat> detectimg;
cv::Mat diffimg, diffimg_hsv, detectimg_on_hsv;
const cv::Scalar greenLED_min(0, 150, 0);
const cv::Scalar greenLED_max(256, 256, 256);
const cv::Scalar blueLED_min(150, 0, 0);
const cv::Scalar blueLED_max(256, 256, 256);
const cv::Scalar HSVLED_min(0, 0, 100);
const cv::Scalar HSVLED_max(256, 256, 256);
double ledmass[4] = { 0 }, ledmomx[4] = { 0 }, ledmomy[4] = { 0 };
const int roi_led_minx_ini[4] = { width, width, width, width },
roi_led_maxx_ini[4] = { 0 }, roi_led_miny_ini[4] = { height, height, height, height }, roi_led_maxy_ini[4] = { 0 };
int roi_led_minx[4], roi_led_maxx[4], roi_led_miny[4], roi_led_maxy[4];
bool leddetected = false;
int ptscnt;
float* ptsptr, * ptscand_ptr, * center_src;
cv::Mat pts, ptscand, labels, centers, afterlabel;
double dist, dist_cluster_thr = 20, dist_centers_thr = 10;
uint8_t* diffimg_src, * detectimg0_src, * detectimg1_src, * detectimg_on_src, * detectimghsv_on_src, * afterlabel_src;
int on_img_id, on_img_cnt, blueno = -1, labelno;
int32_t* labelptr, * labelptr_debug;
int greenbluecnt[4][2] = { 0 };
float maxbluegreenratio = 0;
int32_t h_on;
vector<cv::Rect> rois, rois_rand;
const int roi_led_margin = 10;
double ledimpos[4][2] = { 0 }, ledidimpos[4][2] = { 0 }, ledimpos_rand[4][2] = { 0 };
double ledcog[2] = { 0 };
double thetamax, thetamin, thetamaxid, thetaminid;
double cross, dot, theta[3];
double phi, w, lambda;
double ledcamdir[4][3] = { 0 };
double lednormdir[4][3] = { 0 };
glm::mat4 RTc2m = glm::mat4(1.0), RTuavrs2ugvrs = glm::mat4(1.0);
glm::mat4 RTuavrs2hsc = glm::mat4(1.0), RTugvmk2rs = glm::mat4(1.0);
glm::mat4 RTuavrs2mk = glm::mat4(1.0);
glm::mat4* RTuavrs2ugvrs_buffer, * RTuavrs2ugvrs_toGPU;
int RTuavrs2ugvrs_bufferid = 0, RTuavrs2ugvrs_outid = 0;
cv::Mat A = cv::Mat::zeros(12, 7, CV_64F);
cv::Mat x = cv::Mat::zeros(7, 1, CV_64F);
cv::Mat b = cv::Mat::zeros(12, 1, CV_64F);
double* Asrc = A.ptr<double>(0);
double* xsrc = x.ptr<double>(0);
double* bsrc = b.ptr<double>(0);
const double markeredge = 235 / 2 * 0.001;//単位はm
const double markerpos[4][2] = { {markeredge, markeredge}, {-markeredge, markeredge}, {-markeredge, -markeredge}, {markeredge, -markeredge} };
int detectresult = -1;

/// 単軸ロボットに関する変数
RS232c axisrobot;
#define READBUFFERSIZE 256
char replybuf[READBUFFERSIZE];
char axisrobmodes[][10] = { "@SRVO", "@START", "@ORG" };
char axisrobcommand[READBUFFERSIZE] = "";
const int initaxisstart = 100, initaxisend = 500;
const int posunits = 100, speedunits = 10;



//ログに関するパラメータ
const int timeout = 10;
const int log_img_fps = 70;
const int log_img_fps_hs = 500;
const int log_led_fps = 500;
const int log_img_finish_cnt = log_img_fps * timeout + 100;
const int log_img_finish_cnt_hs = log_img_fps_hs * timeout + 100;
const int log_pose_finish_cnt = log_led_fps * timeout + 100;
long long log_glrsimg_cnt = 0, log_hscimg_cnt = 0, log_pose_cnt = 0;
bool showsavehscimg = false, showsaveglimg = false;
uint8_t* save_img_on_src;
bool saveimgsflg = false, saveimgsstartflg = false;
struct Logs
{
	vector<cv::Mat> in_imgs_log;
	vector<cv::Mat> gl_imgs_log;
	vector<vector<cv::Mat>> rs_imgs_log;
	cv::Mat* in_imgs_log_ptr;
	cv::Mat* gl_imgs_log_ptr;
	double* LED_times;
	double* log_times;
	double* glrslog_times;
	double* glrslog_times_diff;
	double* hsclog_times;
	double* hsclog_times_diff;
	int* LED_results;
	vector<glm::mat4> LED_RTuavrs2ugvrs;
};

//プロトタイプ宣言
void GetPointClouds(realsense* rs, bool* flg, PointCloud* pc);
void TakePicture(kayacoaxpress* cam, bool* flg, Logs* logs);
int DetectLEDMarker();
void ShowImgsHSC(bool* flg, Logs* logs);
void ShowSaveImgsGL(bool* flg, PointCloud** pc_src, Logs* logs);
void Read_Reply_toEND(RS232c* robot);
void wait_QueryPerformance(double finishtime, LARGE_INTEGER freq);
void ControlAxisRobot(RS232c* robot, bool* flg);
void SaveImgHSC(bool* flg, Logs* logs, const int* finishcnt, double* onelooptime);

using namespace std;

#define GETPOINTSREALSENSE_THREAD_
#define GETRELPOSE_THREAD_
#define SHOW_PROCESSING_TIME_
#define SHOW_IMGS_THREAD_
#define SHOW_OPENGL_THREAD_
//#define DEBUG_
#define ROI_MODE_

#define SAVE_IMGS_
//#define SAVE_IMGS_HSC_
//#define SAVE_IMGS_AT_HIGHSPEED_
//#define SAVE_IMGS_REALSENSE_
#define SAVE_HSC2MK_POSE_
//#define MOVE_AXISROBOT_

int main() {
	//パラメータ
	bool flg = true;
	if (!QueryPerformanceFrequency(&freq)) { return 0; }// 単位習得

	//カメラの初期化
	kayacoaxpress cam;
	cam.connect(1);

	//カメラパラメータの設定
	cout << "Set Camera Params..." << endl;
	cam.setParam(paramTypeKAYACoaXpress::AcquisitionMode::TriggerMode, 1); //トリガーモードで起動
	cam.setParam(paramTypeCamera::paramInt::WIDTH, width);
	cam.setParam(paramTypeCamera::paramInt::HEIGHT, height);
	cam.setParam(paramTypeKAYACoaXpress::paramInt::OffsetX, offsetx);
	cam.setParam(paramTypeKAYACoaXpress::paramInt::OffsetY, offsety);
	cam.setParam(paramTypeCamera::paramFloat::FPS, fps);
	cam.setParam(paramTypeKAYACoaXpress::paramFloat::ExposureTime, exposuretime);
	cam.setParam(paramTypeKAYACoaXpress::Gain::x1);
	cam.setParam(paramTypeKAYACoaXpress::CaptureType::BayerGRGrab);
	cam.parameter_all_print();

	//画像取得用のリングバッファの作成
	full = cv::Mat(cam.getParam(paramTypeCamera::paramInt::HEIGHT), cam.getParam(paramTypeCamera::paramInt::WIDTH), CV_8UC3, cv::Scalar::all(255));
	zero = cv::Mat(cam.getParam(paramTypeCamera::paramInt::HEIGHT), cam.getParam(paramTypeCamera::paramInt::WIDTH), CV_8UC3, cv::Scalar::all(0));
	zeromulti = cv::Mat(cam.getParam(paramTypeCamera::paramInt::HEIGHT) * multicnt, cam.getParam(paramTypeCamera::paramInt::WIDTH), CV_8UC3, cv::Scalar::all(0));
	cout << "Set Mat Cycle Buffer..." << endl;
	for (size_t i = 0; i < ringbuffersize; i++)
	{
		in_imgs.push_back(zeromulti.clone());
		processflgs.push_back(false);
	}

	//レーザCalibrationの結果の呼び出し
	FILE* fcam;
	fcam = fopen("202105011655_fisheyeparam.csv", "r");
	for (size_t i = 0; i < 4; i++) { fscanf(fcam, "%lf,", &map_coeff[i]); }
	for (size_t i = 0; i < 4; i++) { fscanf(fcam, "%lf,", &stretch_mat[i]); }
	swap(stretch_mat[1], stretch_mat[2]);
	for (size_t i = 0; i < 2; i++) { fscanf(fcam, "%lf,", &distort[i]); }
	fclose(fcam);
	det = 1 / (stretch_mat[0] - stretch_mat[1] * stretch_mat[2]);

	//PoseCalibration結果の呼び出し
	FILE* fpose;
	fpose = fopen("202105051723_poseparam.csv", "r");
	for (size_t i = 0; i < 3; i++)
	{
		for (size_t j = 0; j < 3; j++)
		{
			fscanf(fpose, "%f,", &RTuavrs2hsc[j][i]);
		}
	}
	for (size_t i = 0; i < 3; i++)
	{
		fscanf(fpose, "%f,", &RTuavrs2hsc[3][i]);
		RTuavrs2hsc[3][i] /= 1000;//単位はm
	}
	for (size_t i = 0; i < 3; i++)
	{
		for (size_t j = 0; j < 3; j++)
		{
			fscanf(fpose, "%f,", &RTugvmk2rs[j][i]);
		}
	}
	for (size_t i = 0; i < 3; i++)
	{
		fscanf(fpose, "%f,", &RTugvmk2rs[3][i]);
		RTugvmk2rs[3][i] /= 1000;//単位はm
	}

	//輝点保存用行列の作成
	ptscand = cv::Mat::zeros(width * height, 1, CV_32FC2);
	ptscand_ptr = ptscand.ptr<float>(0);

	//ROIの設定
	for (size_t i = 0; i < 4; i++)
	{
		rois.push_back(cv::Rect(0, 0, width, height));
		rois_rand.push_back(cv::Rect(0, 0, width, height));
	}

	//LED位置検出のためのMat vector作成
	for (size_t i = 0; i < 2; i++)
	{
		detectimg.push_back(zero.clone());
	}
	diffimg = zero.clone();
	diffimg_src = diffimg.ptr<uint8_t>(0);

	//計算した位置姿勢の保存用リングバッファの作成
	RTuavrs2ugvrs_buffer = (glm::mat4*)malloc(sizeof(glm::mat4) * ringbuffersize);
	for (size_t i = 0; i < ringbuffersize; i++)
	{
		memcpy((RTuavrs2ugvrs_buffer + i), &RTuavrs2ugvrs, sizeof(glm::mat4));
	}

	//PointCloud初期化
	cout << "Set PointCloud buffers....";
	vector<PointCloud> pcs;
	PointCloud* pcs_src[realsense_cnt];

	for (size_t i = 0; i < realsense_cnt; i++)
	{
		PointCloud pc;
		pc.pc_ringbuffer = (float*)malloc(sizeof(float) * ring_size_realsense * vert_cnt * 3);
		pc.texcoords_ringbuffer = (float*)malloc(sizeof(float) * ring_size_realsense * vert_cnt * 2);
		pc.pc_buffer = (rs2::vertex*)malloc(sizeof(float) * vert_cnt * 3);
		pc.texcoords = (rs2::texture_coordinate*)malloc(sizeof(float) * vert_cnt * 2);
		pcs.push_back(pc);
	}
	pcs_src[0] = &pcs[0];
	pcs_src[1] = &pcs[1];

	cout << "OK!" << endl;

	//2つのRealsenseの初期化
	cout << "Set RealsenseD435..........";
	const rs2::device_list device_list = context.query_devices();
	for (rs2::device device : device_list)
	{
		realsense rs(device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER), RS2_FORMAT_BGR8, 
			colorwidth, colorheight, colorfps, RS2_FORMAT_Z16, depthwidth, depthheight, depthfps);
		rs_devices.push_back(rs);
	}
	cout << "OK!" << endl;

	//OpenGL出力の画像と位置姿勢計算の画像保存バッファの作成
	Logs logs;
#ifdef SAVE_IMGS_
	//取得画像を格納するVectorの作成
	std::cout << "Set Img Vector for logs....................";
	cv::Mat gl_img = cv::Mat(window_height, window_width, CV_8UC3, cv::Scalar::all(0));
	cv::Mat rs_img = cv::Mat(colorheight, colorwidth, CV_8UC3, cv::Scalar::all(0));
	for (size_t i = 0; i < log_img_finish_cnt; i++) { logs.gl_imgs_log.push_back(gl_img.clone()); }
#ifdef SAVE_IMGS_HSC_
#ifdef SAVE_IMGS_AT_HIGHSPEED_
	for (size_t i = 0; i < log_img_finish_cnt_hs; i++) { logs.in_imgs_log.push_back(zero.clone()); }
	logs.hsclog_times = (double*)malloc(sizeof(double) * log_img_finish_cnt_hs);
	logs.hsclog_times_diff = (double*)malloc(sizeof(double) * log_img_finish_cnt_hs);
#endif // SAVE_IMGS_HIGHSPEED
#ifndef SAVE_IMGS_AT_HIGHSPEED_
	for (size_t i = 0; i < log_img_finish_cnt; i++) { logs.in_imgs_log.push_back(zero.clone()); }
	logs.hsclog_times = (double*)malloc(sizeof(double) * log_img_finish_cnt);
	logs.hsclog_times_diff = (double*)malloc(sizeof(double) * log_img_finish_cnt);
#endif
#endif // SAVE_IMGS_HSC_
#ifdef SAVE_IMGS_REALSENSE_
	for (size_t i = 0; i < log_img_finish_cnt; i++)
	{
		vector<cv::Mat> rs_imgs;
		for (size_t j = 0; j < realsense_cnt; j++)
		{
			rs_imgs.push_back(rs_img.clone());
		}
		logs.rs_imgs_log.push_back(rs_imgs);
	}
#endif // SAVE_IMGS_REALSENSE_

	logs.in_imgs_log_ptr = logs.in_imgs_log.data();
	logs.gl_imgs_log_ptr = logs.gl_imgs_log.data();
	logs.glrslog_times = (double*)malloc(sizeof(double) * log_img_finish_cnt);
	logs.glrslog_times_diff = (double*)malloc(sizeof(double) * log_img_finish_cnt);
	
	cout << "OK!" << endl;
#endif // SAVE_IMGS_
#ifdef SAVE_HSC2MK_POSE_
	cout << "Set Pose vector for logs........................";
	logs.LED_times = (double*)malloc(sizeof(double) * log_pose_finish_cnt);
	logs.log_times = (double*)malloc(sizeof(double) * log_pose_finish_cnt);
	logs.LED_results = (int*)malloc(sizeof(int) * log_pose_finish_cnt);
	for (size_t i = 0; i < log_pose_finish_cnt; i++)
	{
		logs.LED_RTuavrs2ugvrs.push_back(glm::mat4(1.0));
	}
	cout << "OK!" << endl;
#endif // SAVE_HSC2MK_POSE_

	//単軸ロボットとの通信確保
#ifdef MOVE_AXISROBOT_
	std::cout << "Set commection to AXIS ROBOT...............";
	if (!axisrobot.Connect("COM6", 38400, 8, ODDPARITY, 0, 0, 0, 20000, 20000)) {
		cout << "No connect" << endl;
		return 1;
	}
	std::cout << "OK!" << endl;

	snprintf(axisrobcommand, READBUFFERSIZE, "%s%d.1\r\n", axisrobmodes[0], 1);
	axisrobot.Send(axisrobcommand);
	memset(axisrobcommand, '\0', READBUFFERSIZE);
	Read_Reply_toEND(&axisrobot);
	std::cout << "SERVO ON" << endl;
	wait_QueryPerformance(0.1, freq);
	snprintf(axisrobcommand, READBUFFERSIZE, "%s.1\r\n", axisrobmodes[2]);
	axisrobot.Send(axisrobcommand);
	cout << "ORG START" << endl;
	memset(axisrobcommand, '\0', READBUFFERSIZE);
	Read_Reply_toEND(&axisrobot);
	std::cout << "ORG STOP" << endl;
#endif // MOVE_AXISROBOT_


	//カメラ起動
	cout << "Camera Start!" << endl;
	cam.start();

	//スレッド作成
#ifdef GETPOINTSREALSENSE_THREAD_
	vector<thread> GetPointsThread;
	for (size_t i = 0; i < realsense_cnt; i++)
	{
		GetPointsThread.push_back(thread(GetPointClouds, &rs_devices[i], &flg, &pcs[i]));
	}
#endif // GETPOINTREALSENSE_THREAD_
	thread TakePictureThread(TakePicture, &cam, &flg, &logs);
#ifdef SHOW_IMGS_THREAD_
	thread ShowImgsHSCThread(ShowImgsHSC, &flg, &logs);
#endif // SHOW_	
	thread ShowSaveImgsGLThread(ShowSaveImgsGL, &flg, pcs_src, &logs);
#ifdef MOVE_AXISROBOT_
	thread MoveAxisRobotThread(ControlAxisRobot, &axisrobot, &flg);
#endif // MOVE_AXISROBOT_
#ifdef SAVE_IMGS_HSC_
#ifdef SAVE_IMGS_AT_HIGHSPEED_
	thread SaveHSCImgsThread(SaveImgHSC, &flg, &logs, &log_img_finish_cnt_hs, &takepic_time);
#endif // SAVE_IMGS_AT_HIGHSPEED_
#ifndef SAVE_IMGS_AT_HIGHSPEED_
	thread SaveHSCImgsThread(SaveImgHSC, &flg, &logs, &log_img_finish_cnt, &showhsc_time);
#endif // !SAVE_IMGS_AT_HIGHSPEED_
#endif // SAVE_IMGS_HSC_






	//メインループ
	cout << "Main loop start!" << endl;
	QueryPerformanceCounter(&start);

	while (timer < 1.5)
	{
		QueryPerformanceCounter(&stop);
		timer = (double)(stop.QuadPart - start.QuadPart) / freq.QuadPart;
	}
	while (flg)
	{
		QueryPerformanceCounter(&detectstart);
#ifdef GETRELPOSE_THREAD_
		detectresult = DetectLEDMarker();
#endif // GETRELPOSE_THREAD_

		QueryPerformanceCounter(&detectend);
		detecttime = (double)(detectend.QuadPart - detectstart.QuadPart) / freq.QuadPart;
		while (detecttime < calcpose_time)
		{
			QueryPerformanceCounter(&detectend);
			detecttime = (double)(detectend.QuadPart - detectstart.QuadPart) / freq.QuadPart;
		}

		if (detectresult == 0)
		{	//ここに位置姿勢推定結果を反映させる計算を実装
			memcpy((RTuavrs2ugvrs_buffer + RTuavrs2ugvrs_bufferid), &RTuavrs2ugvrs, sizeof(glm::mat4));
			RTuavrs2ugvrs_bufferid = (RTuavrs2ugvrs_bufferid + 1) % ringbuffersize;
		}

		if (saveimgsflg)
		{
			//ログ時間計測
			QueryPerformanceCounter(&logend);
			logtime = (double)(logend.QuadPart - logstart.QuadPart) / freq.QuadPart;
			if (logtime > timeout) flg = false;

#ifdef SAVE_HSC2MK_POSE_
			//位置姿勢ログ保存
			* (logs.LED_results + log_pose_cnt) = detectresult;
			* (logs.LED_times + log_pose_cnt) = detecttime;
			*(logs.log_times + log_pose_cnt) = logtime;
			logs.LED_RTuavrs2ugvrs[log_pose_cnt] = RTuavrs2ugvrs;
			log_pose_cnt++;
			if (log_pose_cnt > log_pose_finish_cnt) flg = false;
#endif // SAVE_HSC2MK_POSE_
		}

#ifdef SHOW_PROCESSING_TIME_
		cout << "DetectLEDMarker() result: " << detectresult << endl;
		std::cout << "DetectLEDMarker() time: " << detecttime << endl;
#endif // SHOW_PROCESSING_TIME_
	}

	//スレッド削除
#ifdef GETPOINTSREALSENSE_THREAD_
	for (size_t i = 0; i < GetPointsThread.size(); i++)
	{
		if (GetPointsThread[i].joinable())GetPointsThread[i].join();
	}
#endif // GETPOINTSREALSENSE_THREAD_
	if (TakePictureThread.joinable())TakePictureThread.join();
#ifdef SHOW_IMGS_THREAD_
	if (ShowImgsHSCThread.joinable())ShowImgsHSCThread.join();
#endif // SHOW_IMGS_THREAD_
	if (ShowSaveImgsGLThread.joinable())ShowSaveImgsGLThread.join(); 
#ifdef MOVE_AXISROBOT_
	if (MoveAxisRobotThread.joinable())MoveAxisRobotThread.join();
#endif // MOVE_AXISROBOT_
#ifdef SAVE_IMGS_HSC_
#ifdef SAVE_IMGS_AT_HIGHSPEED_
	if (SaveHSCImgsThread.joinable())SaveHSCImgsThread.join();
#endif // SAVE_IMGS_AT_HIGHSPEED_
#ifndef SAVE_IMGS_AT_HIGHSPEED_
	if (SaveHSCImgsThread.joinable())SaveHSCImgsThread.join();
#endif // !SAVE_IMGS_AT_HIGHSPEED_

#endif // SAVE_IMGS_HSC_



	//カメラの停止
	cam.stop();
	cam.disconnect();

	//単軸ロボットの停止
#ifdef MOVE_AXISROBOT_
	snprintf(axisrobcommand, READBUFFERSIZE, "%s%d.1\r\n", axisrobmodes[0], 0);
	axisrobot.Send(axisrobcommand);
	Read_Reply_toEND(&axisrobot);
	memset(axisrobcommand, '\0', READBUFFERSIZE);
	cout << "SERVO OFF" << endl;
#endif // MOVE_AXISROBOT_

	//ログ保存のための準備
	FILE* fr;
	time_t timer;
	struct tm now;
	timer = time(NULL);
	localtime_s(&now, &timer);
	char dir[256];
	char picdir[256];
	char logfile[256];
	struct stat statBuf;

	//取得した位置姿勢の保存
#ifdef SAVE_HSC2MK_POSE_
	if (log_pose_cnt > 0)
	{
		cout << "Saving poses.." << endl;
		//HSC2MKの位置姿勢保存
		strftime(dir, 256, "D:/Github_output/SuperImposition/MultiSuperImposition_withLEDMarker/results/%y%m%d", &now);
		if (stat(dir, &statBuf) != 0) {
			if (_mkdir(dir) != 0) { return 0; }
		}
		strftime(dir, 256, "D:/Github_output/SuperImposition/MultiSuperImposition_withLEDMarker/results/%y%m%d/%H%M%S", &now);
		if (stat(dir, &statBuf) != 0) {
			if (_mkdir(dir) != 0) { return 0; }
		}
		strftime(logfile, 256, "D:/Github_output/SuperImposition/MultiSuperImposition_withLEDMarker/results/%y%m%d/%H%M%S/LEDpose_results.csv", &now);
		fr = fopen(logfile, "w");
		for (size_t i = 0; i < log_pose_cnt; i++)
		{
			fprintf(fr, "%lf,", logs.log_times[i]);
			fprintf(fr, "%d,", logs.LED_results[i]);
			fprintf(fr, "%lf,", logs.LED_times[i]);
			for (size_t j = 0; j < 3; j++)
			{
				for (size_t k = 0; k < 3; k++)
				{
					fprintf(fr, "%f,", logs.LED_RTuavrs2ugvrs[i][k][j]);
				}
			}
			for (size_t j = 0; j < 3; j++)
			{
				fprintf(fr, "%f,", logs.LED_RTuavrs2ugvrs[i][3][j]);
			}
			fprintf(fr, "\n");
		}
		fclose(fr);
	}
#endif // SAVE_HSC2MK_POSE_


	//取得した画像の保存
#ifdef SAVE_IMGS_
	if (log_glrsimg_cnt > 0)
	{
		std::cout << "Saving imgs..." << endl;
		//HSCの画像保存
		strftime(picdir, 256, "D:/Github_output/SuperImposition/MultiSuperImposition_withLEDMarker/results/%y%m%d", &now);
		if (stat(picdir, &statBuf) != 0) {
			if (_mkdir(picdir) != 0) { return 0; }
		}
		strftime(picdir, 256, "D:/Github_output/SuperImposition/MultiSuperImposition_withLEDMarker/results/%y%m%d/%H%M%S", &now);
		if (stat(picdir, &statBuf) != 0) {
			if (_mkdir(picdir) != 0) { return 0; }
		}
		strftime(picdir, 256, "D:/Github_output/SuperImposition/MultiSuperImposition_withLEDMarker/results/%y%m%d/%H%M%S/HSC", &now);
		if (stat(picdir, &statBuf) != 0) {
			if (_mkdir(picdir) != 0) { return 0; }
		}
		char picturename[256];
		char picsubname[256];
		strftime(picsubname, 256, "D:/Github_output/SuperImposition/MultiSuperImposition_withLEDMarker/results/%y%m%d/%H%M%S/HSC/frame", &now);
		for (int i = 0; i < log_hscimg_cnt; i++)
		{
			sprintf(picturename, "%s%05d.png", picsubname, i);//png可逆圧縮
			cv::cvtColor(logs.in_imgs_log[i], logsaveimg, CV_RGB2BGR);
			cv::imwrite(picturename, logsaveimg);
		}
		//HSCの画像保存時刻の保存
		strftime(logfile, 256, "D:/Github_output/SuperImposition/MultiSuperImposition_withLEDMarker/results/%y%m%d/%H%M%S/HSCimg_times.csv", &now);
		fr = fopen(logfile, "w");
		for (size_t i = 0; i < log_hscimg_cnt; i++)
		{
			fprintf(fr, "%lf,", logs.hsclog_times[i]);
			fprintf(fr, "%lf\n", logs.hsclog_times_diff[i]);
		}
		fclose(fr);
		//OpenGLの画像保存
		strftime(picdir, 256, "D:/Github_output/SuperImposition/MultiSuperImposition_withLEDMarker/results/%y%m%d/%H%M%S/GL", &now);
		if (stat(picdir, &statBuf) != 0) {
			if (_mkdir(picdir) != 0) { return 0; }
		}
		strftime(picsubname, 256, "D:/Github_output/SuperImposition/MultiSuperImposition_withLEDMarker/results/%y%m%d/%H%M%S/GL/frame", &now);
		for (int i = 0; i < log_glrsimg_cnt; i++)
		{
			sprintf(picturename, "%s%05d.png", picsubname, i);//png可逆圧縮
			cv::flip(logs.gl_imgs_log[i], logs.gl_imgs_log[i], 0);
			cv::imwrite(picturename, logs.gl_imgs_log[i]);
		}
#ifdef SAVE_IMGS_REALSENSE_
		//RealSenseの画像保存
		for (int i = 0; i < realsense_cnt; i++)
		{
			strftime(picdir, 256, "D:/Github_output/SuperImposition/MultiSuperImposition_withLEDMarker/results/%y%m%d/%H%M%S/RS", &now);
			char rsnum[10];
			sprintf(rsnum, "%d", i);
			strcat(picdir, rsnum);
			if (stat(picdir, &statBuf) != 0) {
				if (_mkdir(picdir) != 0) { return 0; }
			}
			for (int j = 0; j < log_glrsimg_cnt; j++)
			{
				sprintf(picturename, "%s/frame%05d.png", picdir, j);
				cv::imwrite(picturename, logs.rs_imgs_log[j][i]);
			}
		}
#endif // SAVE_IMGS_REALSENSE_		
		//RealSenseとOpenGLの画像保存時刻の保存
		strftime(logfile, 256, "D:/Github_output/SuperImposition/MultiSuperImposition_withLEDMarker/results/%y%m%d/%H%M%S/RSGLimg_times.csv", &now);
		fr = fopen(logfile, "w");
		for (size_t i = 0; i < log_glrsimg_cnt; i++)
		{
			fprintf(fr, "%lf,", logs.glrslog_times[i]);
			fprintf(fr, "%lf\n", logs.glrslog_times_diff[i]);
		}
		fclose(fr);
		std::cout << "Imgs finished!" << endl;
	}
	
#endif // SAVE_IMGS_


	return 0;
}

//画像を格納する
void TakePicture(kayacoaxpress* cam, bool* flg, Logs* logs) {
	while (*flg)
	{
		QueryPerformanceCounter(&takestart);
		takepicid = in_imgs_saveid % ringbuffersize;
		in_img_multi_src = in_imgs[takepicid].ptr<uint8_t>(0);

		cam->captureFrame2(in_img_multi_src, multicnt);
		QueryPerformanceCounter(&takeend);
		taketime = (double)(takeend.QuadPart - takestart.QuadPart) / freq.QuadPart;
		while (taketime < takepic_time)
		{
			QueryPerformanceCounter(&takeend);
			taketime = (double)(takeend.QuadPart - takestart.QuadPart) / freq.QuadPart;
		}
		in_imgs_saveid = (in_imgs_saveid + 1) % ringbuffersize;
		processflgs[takepicid] = true;		
		
#ifdef SHOW_PROCESSING_TIME_
		std::cout << "TakePicture() time: " << taketime << endl;
#endif // SHOW_PROCESSING_TIME_
	}
}

void SaveImgHSC(bool* flg, Logs* logs, const int* finishcnt, double* onelooptime) {
	while (*flg)
	{
		QueryPerformanceCounter(&savehscstart);
		//sを押して画像保存開始
		if (saveimgsflg)
		{
			//LED画像の保存
			save_img_on_src = in_imgs[(takepicid - 1 + ringbuffersize) % ringbuffersize].ptr<uint8_t>(0);
			memcpy((logs->in_imgs_log_ptr + log_hscimg_cnt)->data, save_img_on_src, height * width * 3);
			//HSCの画像取得時間計測
			QueryPerformanceCounter(&hsclogend);
			hsclogtime = (double)(hsclogend.QuadPart - logstart.QuadPart) / freq.QuadPart;
			*(logs->hsclog_times + log_hscimg_cnt) = hsclogtime;
			*(logs->hsclog_times_diff + log_hscimg_cnt) = taketime;

			log_hscimg_cnt++;
			if (log_hscimg_cnt > *finishcnt) *flg = false;
		}
		QueryPerformanceCounter(&savehscend);
		savehsctime = (double)(savehscend.QuadPart - savehscstart.QuadPart) / freq.QuadPart;
		while (savehsctime < *onelooptime)
		{
			QueryPerformanceCounter(&savehscend);
			savehsctime = (double)(savehscend.QuadPart - savehscstart.QuadPart) / freq.QuadPart;
		}
	}
}

//画像の点群全てを表示
void ShowImgsHSC(bool* flg, Logs* logs) {
	QueryPerformanceCounter(&hscstart);
	while (hsctime < 1.5)
	{
		QueryPerformanceCounter(&hscend);
		hsctime = (double)(hscend.QuadPart - hscstart.QuadPart) / freq.QuadPart;
	}
	while (*flg)
	{
		QueryPerformanceCounter(&hscstart);
		showsavehscimg = false;

		//OpenCVで画像表示
		cv::cvtColor(in_imgs[(in_imgs_saveid - 2 + ringbuffersize) % ringbuffersize], show_img, CV_RGB2BGR);
		cv::imshow("img", show_img);
		int key = cv::waitKey(1);
		if (key == 'q') *flg = false;

#ifdef SAVE_IMGS_
		//sを押すと，画像保存が開始される
		if (key == 's' && !saveimgsstartflg) {
			saveimgsflg = true;
			QueryPerformanceCounter(&logstart);
			saveimgsstartflg = true;
		}
#endif // SAVE_IMGS_


		QueryPerformanceCounter(&hscend);
		hsctime = (double)(hscend.QuadPart - hscstart.QuadPart) / freq.QuadPart;
		while (hsctime < showhsc_time)
		{
			QueryPerformanceCounter(&hscend);
			hsctime = (double)(hscend.QuadPart - hscstart.QuadPart) / freq.QuadPart;
		}
		showsavehscimg = true;
		while (!showsaveglimg)
		{
			QueryPerformanceCounter(&hscend);
			hsctime = (double)(hscend.QuadPart - hscstart.QuadPart) / freq.QuadPart;
		}
		
#ifdef SHOW_PROCESSING_TIME_
		std::cout << "ShowImgsHSC() time: " << hsctime << endl;
#endif // SHOW_PROCESSING_TIME_
	}
}

//HSCの画像とOpenGL表示の画像を保存
void ShowSaveImgsGL(bool* flg, PointCloud** pc_src, Logs* logs) {
	//OpenGLの初期化
	initGL();

	QueryPerformanceCounter(&glstart);
	while (gltime < 1.5)
	{
		QueryPerformanceCounter(&glend);
		gltime = (double)(glend.QuadPart - glstart.QuadPart) / freq.QuadPart;
	}
	while (*flg)
	{
		QueryPerformanceCounter(&glstart);
		showsaveglimg = false;

#ifdef SHOW_OPENGL_THREAD_
		//OpenGLで2台のRealsenseの点群出力
		for (size_t i = 0; i < realsense_cnt; i++)
		{
			getpc_id = pc_src[i]->pc_ringid - 1;
			if (getpc_id < 0) getpc_id += ring_size_realsense;
			gl_pc_src[i] = pc_src[i]->pc_ringbuffer + (unsigned long long)getpc_id * 3 * vert_cnt;
			gl_texcoord_src[i] = pc_src[i]->texcoords_ringbuffer + (unsigned long long)getpc_id * 2 * vert_cnt;
			gl_tex_src[i] = pc_src[i]->colorframe_buffer + getpc_id;
		}
		RTuavrs2ugvrs_outid = (RTuavrs2ugvrs_bufferid - 1 + ringbuffersize) % ringbuffersize;
		RTuavrs2ugvrs_toGPU = RTuavrs2ugvrs_buffer + RTuavrs2ugvrs_outid;
		drawGL_realsense(gl_pc_src, gl_texcoord_src, gl_tex_src, RTuavrs2ugvrs_toGPU);
#endif // SHOW_OPENGL_



		QueryPerformanceCounter(&glend);
		gltime = (double)(glend.QuadPart - glstart.QuadPart) / freq.QuadPart;
		while (gltime < showgl_time)
		{
			QueryPerformanceCounter(&glend);
			gltime = (double)(glend.QuadPart - glstart.QuadPart) / freq.QuadPart;
		}
		showsaveglimg = true;
		//while (!showsavehscimg)
		//{
		//	QueryPerformanceCounter(&glend);
		//	gltime = (double)(glend.QuadPart - glstart.QuadPart) / freq.QuadPart;
		//}
#ifdef SHOW_PROCESSING_TIME_
		std::cout << "ShowSaveImgsGL() time: " << gltime << endl;
#endif // SHOW_PROCESSING_TIME_

#ifdef SAVE_IMGS_
		//sを押して画像保存開始
		if (saveimgsflg)
		{
			//OpenGL表示の画像保存
			saveImgCV((logs->gl_imgs_log_ptr + log_glrsimg_cnt)->data);

#ifdef SAVE_IMGS_REALSENSE_
			//RealSenseの画像保存
			for (size_t i = 0; i < realsense_cnt; i++)
			{
				memcpy(logs->rs_imgs_log[log_glrsimg_cnt][i].data, gl_tex_src[i]->get_data(), colorwidth * colorheight * 3);
			}
#endif // SAVE_IMGS_REALSENSE_

			//OpenGLとRealSenseの画像取得時間計測
			QueryPerformanceCounter(&glrslogend);
			glrslogtime = (double)(glrslogend.QuadPart - logstart.QuadPart) / freq.QuadPart;
			*(logs->glrslog_times + log_glrsimg_cnt) = glrslogtime;
			*(logs->glrslog_times_diff + log_glrsimg_cnt) = gltime;

			log_glrsimg_cnt++;
			if (log_glrsimg_cnt > log_img_finish_cnt) *flg = false;
		}
#endif // SAVE_IMGS_
	}
	//OpenGLの終了
	finishGL();
}

//RealSenseから点群や画像の取得
void GetPointClouds(realsense* rs, bool* flg, PointCloud* pc) {
	while (*flg)
	{
		//点群計算
		rs->update_frame();
		rs->update_color();
		rs->update_depth();
		rs->calc_pointcloud();
		pc->pc_buffer = rs->points.get_vertices();
		pc->texcoords = rs->points.get_texture_coordinates();
		//取得点群をリングバッファに保存
		memcpy((pc->pc_ringbuffer + (unsigned long long)pc->pc_ringid * vert_cnt * 3), pc->pc_buffer, sizeof(float) * vert_cnt * 3);
		memcpy((pc->texcoords_ringbuffer + (unsigned long long)pc->pc_ringid * vert_cnt * 2), pc->texcoords, sizeof(float) * vert_cnt * 2);
		pc->colorframe_buffer[pc->pc_ringid] = rs->colorframe;

		//リングバッファの番号を更新
		pc->pc_ringid = (pc->pc_ringid + 1) % ring_size_realsense;
	}
}

//LED markerから位置姿勢計算
int DetectLEDMarker() {
#ifdef DEBUG_
	QueryPerformanceCounter(&detectstartdebug);
#endif // 

	//画像の格納
	detectid = (in_imgs_saveid - 1 + ringbuffersize) % ringbuffersize;
	detectimg_multi_src = in_imgs[detectid].ptr<uint8_t>(0);
	if (processflgs[detectid])
	{
		memcpy(detectimg[0].data, detectimg_multi_src, height * width * 3);
		memcpy(detectimg[1].data, detectimg_multi_src + height * width * 3, height * width * 3);
	}
#ifdef DEBUG_
	QueryPerformanceCounter(&detectend);
	detecttimef = (double)(detectend.QuadPart - detectstartdebug.QuadPart) / freq.QuadPart;
#endif // SHOW_PROCESSING_TIME_
#ifdef DEBUG_
	QueryPerformanceCounter(&detectstartdebug);
#endif // SHOW_PROCESSING_TIME_
	//LEDが未検出の時は，画像全体を探索する
	if (detectimg[0].data != NULL && detectimg[1].data != NULL && (int)detectimg[0].data[0] != 255 && (int)detectimg[1].data[0] != 255)
	{
		//クラスターごとに輝度重心を計算する
		for (size_t i = 0; i < 4; i++)
		{
			ledmass[i] = 0, ledmomx[i] = 0, ledmomy[i] = 0;
		}
		memcpy(roi_led_minx, roi_led_minx_ini, sizeof(roi_led_minx_ini));
		memcpy(roi_led_maxx, roi_led_maxx_ini, sizeof(roi_led_maxx_ini));
		memcpy(roi_led_miny, roi_led_miny_ini, sizeof(roi_led_miny_ini));
		memcpy(roi_led_maxy, roi_led_maxy_ini, sizeof(roi_led_maxy_ini));

		//4つ全てのLEDを検出していない時
		if (!(leddetected))
		{
			//差分画像の生成
			cv::absdiff(detectimg[0], detectimg[1], diffimg);
			//HSVのVの閾値処理
			diffimg_src = diffimg.ptr<uint8_t>(0);
			ptscnt = 0;
			for (size_t i = 0; i < width; i++)
			{
				for (size_t j = 0; j < height; j++)
				{
					if ((uint8_t)diffimg_src[j * width * 3 + i * 3] > HSVLED_min(2) || (uint8_t)diffimg_src[j * width * 3 + i * 3 + 1] > HSVLED_min(2) || (uint8_t)diffimg_src[j * width * 3 + i * 3 + 2] > HSVLED_min(2))
					{
						ptscand_ptr[ptscnt * 2 + 0] = (float)i;
						ptscand_ptr[ptscnt * 2 + 1] = (float)j;
						ptscnt++;
					}
				}
			}
			//ここで差分画像から輝点が見つからないときの例外処理を書く
			if (ptscnt <= 0) {
				processflgs[detectid] = false;
				return 6;
			}
#ifdef DEBUG_
			QueryPerformanceCounter(&detectend);
			detecttimea = (double)(detectend.QuadPart - detectstartdebug.QuadPart) / freq.QuadPart;
#endif // SHOW_PROCESSING_TIME_

#ifdef DEBUG_
			QueryPerformanceCounter(&detectstartdebug);
#endif // SHOW_PROCESSING_TIME_
			//輝度の高い点群を4か所にクラスタリング
			pts = ptscand(cv::Rect(0, 0, 1, ptscnt));
			cv::kmeans(pts, 4, labels, cvTermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1.0), 1, cv::KMEANS_PP_CENTERS, centers);
			center_src = centers.ptr<float>(0);
			//クラスタ間の距離が閾値以下ならば未検出と判定
			for (size_t i = 0; i < 4; i++)
			{
				for (size_t j = 0; j < 4; j++)
				{
					if (i != j)
					{
						dist = hypot(center_src[i * 2 + 0] - center_src[j * 2 + 0], center_src[i * 2 + 1] - center_src[j * 2 + 1]);
						if (dist < dist_centers_thr) {
							processflgs[detectid] = false;
							return 1;
						}
					}
				}
			}
#ifdef DEBUG_
			QueryPerformanceCounter(&detectend);
			detecttimeb = (double)(detectend.QuadPart - detectstartdebug.QuadPart) / freq.QuadPart;
#endif // SHOW_PROCESSING_TIME_



#ifdef DEBUG_
			//デバッグ:分類結果の確認
			afterlabel = cv::Mat(896, 896, CV_8UC3, cv::Scalar::all(0));
			afterlabel_src = afterlabel.ptr<uint8_t>(0);
			labelptr_debug = labels.ptr<int32_t>(0);
			for (size_t i = 0; i < ptscnt; i++)
			{
				//cout << (int32_t)labelptr[i] << endl;
				if ((int32_t)labelptr_debug[i] == 0)
				{//赤
					afterlabel_src[(int)ptscand_ptr[i * 2 + 1] * width * 3 + (int)ptscand_ptr[i * 2 + 0] * 3] = 60;
					afterlabel_src[(int)ptscand_ptr[i * 2 + 1] * width * 3 + (int)ptscand_ptr[i * 2 + 0] * 3 + 1] = 20;
					afterlabel_src[(int)ptscand_ptr[i * 2 + 1] * width * 3 + (int)ptscand_ptr[i * 2 + 0] * 3 + 2] = 220;
				}
				else if ((int32_t)labelptr_debug[i] == 1)
				{//黄色
					afterlabel_src[(int)ptscand_ptr[i * 2 + 1] * width * 3 + (int)ptscand_ptr[i * 2 + 0] * 3] = 0;
					afterlabel_src[(int)ptscand_ptr[i * 2 + 1] * width * 3 + (int)ptscand_ptr[i * 2 + 0] * 3 + 1] = 215;
					afterlabel_src[(int)ptscand_ptr[i * 2 + 1] * width * 3 + (int)ptscand_ptr[i * 2 + 0] * 3 + 2] = 255;
				}
				else if ((int32_t)labelptr_debug[i] == 2)
				{//緑
					afterlabel_src[(int)ptscand_ptr[i * 2 + 1] * width * 3 + (int)ptscand_ptr[i * 2 + 0] * 3] = 127;
					afterlabel_src[(int)ptscand_ptr[i * 2 + 1] * width * 3 + (int)ptscand_ptr[i * 2 + 0] * 3 + 1] = 255;
					afterlabel_src[(int)ptscand_ptr[i * 2 + 1] * width * 3 + (int)ptscand_ptr[i * 2 + 0] * 3 + 2] = 0;
				}
				else if ((int32_t)labelptr_debug[i] == 3)
				{//青
					afterlabel_src[(int)ptscand_ptr[i * 2 + 1] * width * 3 + (int)ptscand_ptr[i * 2 + 0] * 3] = 255;
					afterlabel_src[(int)ptscand_ptr[i * 2 + 1] * width * 3 + (int)ptscand_ptr[i * 2 + 0] * 3 + 1] = 191;
					afterlabel_src[(int)ptscand_ptr[i * 2 + 1] * width * 3 + (int)ptscand_ptr[i * 2 + 0] * 3 + 2] = 0;
				}
			}
#endif // DEBUG

#ifdef DEBUG_
			QueryPerformanceCounter(&detectstartdebug);
#endif // SHOW_PROCESSING_TIME_
			//ON画像がどちらか判定する
			detectimg0_src = detectimg[0].ptr<uint8_t>(0);
			detectimg1_src = detectimg[1].ptr<uint8_t>(0);
			on_img_cnt = 0;
			for (size_t i = 0; i < ptscnt; i++)
			{
				if ((int32_t)detectimg0_src[(int)ptscand_ptr[i * 2 + 1] * width * 3 + (int)ptscand_ptr[i * 2 + 0] * 3] > (int32_t)detectimg1_src[(int)ptscand_ptr[i * 2 + 1] * width * 3 + (int)ptscand_ptr[i * 2 + 0] * 3])
				{//2枚の画像で輝度値を比較
					on_img_cnt++;
				}
			}
			if (on_img_cnt > ptscnt / 2) on_img_id = 0;
			else on_img_id = 1;
			detectimg_on_src = detectimg[on_img_id].ptr<uint8_t>(0);

			//分類ごとに青緑の個数のカウント
			blueno = -1;
			cv::cvtColor(detectimg[on_img_id], detectimg_on_hsv, CV_RGB2HSV);
			detectimghsv_on_src = detectimg_on_hsv.ptr<uint8_t>(0);
			labelptr = labels.ptr<int32_t>(0);
			memset(greenbluecnt, 0, sizeof(int) * 4 * 2);
			maxbluegreenratio = 0;
			for (size_t i = 0; i < ptscnt; i++)
			{
				h_on = (int32_t)detectimghsv_on_src[(int)ptscand_ptr[i * 2 + 1] * width * 3 + (int)ptscand_ptr[i * 2 + 0] * 3];
				if ((int32_t)labelptr[i] == 0)
				{
					if (h_on > 30 && h_on < 90) greenbluecnt[0][0]++;
					else if (h_on > 90 && h_on < 150) greenbluecnt[0][1]++;

				}
				else if ((int32_t)labelptr[i] == 1)
				{
					if (h_on > 30 && h_on < 90) greenbluecnt[1][0]++;
					else if (h_on > 90 && h_on < 150) greenbluecnt[1][1]++;
				}
				else if ((int32_t)labelptr[i] == 2)
				{
					if (h_on > 30 && h_on < 90) greenbluecnt[2][0]++;
					else if (h_on > 90 && h_on < 150) greenbluecnt[2][1]++;
				}
				else if ((int32_t)labelptr[i] == 3)
				{
					if (h_on > 30 && h_on < 90) greenbluecnt[3][0]++;
					else if (h_on > 90 && h_on < 150) greenbluecnt[3][1]++;
				}
			}

			for (size_t i = 0; i < 4; i++)
			{
				//cout << greenbluecnt[i][0] << ", " << greenbluecnt[i][1] << endl;
				if (maxbluegreenratio < (float)greenbluecnt[i][1] / greenbluecnt[i][0]) {
					blueno = (int)i;
					maxbluegreenratio = (float)greenbluecnt[i][1] / greenbluecnt[i][0];
				}
			}
			if (blueno == -1) {
				processflgs[detectid] = false;
				return 5;
			}

			//クラスタごとに輝度重心の計算
			for (size_t i = 0; i < ptscnt; i++)
			{
				labelno = (int)labelptr[i];
				dist = hypot(center_src[labelno * 2 + 0] - (int)ptscand_ptr[i * 2 + 0], center_src[labelno * 2 + 1] - (int)ptscand_ptr[i * 2 + 1]);
				if (dist < dist_cluster_thr)
				{
					if (labelno == blueno)
					{
						if ((int32_t)detectimg_on_src[(int)ptscand_ptr[i * 2 + 1] * width * 3 + (int)ptscand_ptr[i * 2 + 0] * 3 + 2] > blueLED_min(0))
						{//On画像の青の閾値はもっと高い
							ledmass[labelno] += (double)detectimg_on_src[(int)ptscand_ptr[i * 2 + 1] * width * 3 + (int)ptscand_ptr[i * 2 + 0] * 3 + 2];
							ledmomx[labelno] += (double)detectimg_on_src[(int)ptscand_ptr[i * 2 + 1] * width * 3 + (int)ptscand_ptr[i * 2 + 0] * 3 + 2] * (int)ptscand_ptr[i * 2 + 0];
							ledmomy[labelno] += (double)detectimg_on_src[(int)ptscand_ptr[i * 2 + 1] * width * 3 + (int)ptscand_ptr[i * 2 + 0] * 3 + 2] * (int)ptscand_ptr[i * 2 + 1];
							//ROIも計算
							if (roi_led_maxx[labelno] < (int)ptscand_ptr[i * 2 + 0]) roi_led_maxx[labelno] = (int)ptscand_ptr[i * 2 + 0];
							if (roi_led_minx[labelno] > (int)ptscand_ptr[i * 2 + 0]) roi_led_minx[labelno] = (int)ptscand_ptr[i * 2 + 0];
							if (roi_led_maxy[labelno] < (int)ptscand_ptr[i * 2 + 1]) roi_led_maxy[labelno] = (int)ptscand_ptr[i * 2 + 1];
							if (roi_led_miny[labelno] > (int)ptscand_ptr[i * 2 + 1]) roi_led_miny[labelno] = (int)ptscand_ptr[i * 2 + 1];
						}
					}
					else
					{//On画像の緑の閾値はもっと高い
						if ((int32_t)detectimg_on_src[(int)ptscand_ptr[i * 2 + 1] * width * 3 + (int)ptscand_ptr[i * 2 + 0] * 3 + 1] > greenLED_min(1))
						{
							ledmass[labelno] += (double)detectimg_on_src[(int)ptscand_ptr[i * 2 + 1] * width * 3 + (int)ptscand_ptr[i * 2 + 0] * 3 + 1];
							ledmomx[labelno] += (double)detectimg_on_src[(int)ptscand_ptr[i * 2 + 1] * width * 3 + (int)ptscand_ptr[i * 2 + 0] * 3 + 1] * (int)ptscand_ptr[i * 2 + 0];
							ledmomy[labelno] += (double)detectimg_on_src[(int)ptscand_ptr[i * 2 + 1] * width * 3 + (int)ptscand_ptr[i * 2 + 0] * 3 + 1] * (int)ptscand_ptr[i * 2 + 1];
							//ROIも計算
							if (roi_led_maxx[labelno] < (int)ptscand_ptr[i * 2 + 0]) roi_led_maxx[labelno] = (int)ptscand_ptr[i * 2 + 0];
							if (roi_led_minx[labelno] > (int)ptscand_ptr[i * 2 + 0]) roi_led_minx[labelno] = (int)ptscand_ptr[i * 2 + 0];
							if (roi_led_maxy[labelno] < (int)ptscand_ptr[i * 2 + 1]) roi_led_maxy[labelno] = (int)ptscand_ptr[i * 2 + 1];
							if (roi_led_miny[labelno] > (int)ptscand_ptr[i * 2 + 1]) roi_led_miny[labelno] = (int)ptscand_ptr[i * 2 + 1];
						}
					}
				}
			}
			for (size_t i = 0; i < 4; i++)
			{
				if (roi_led_maxx[i] > width - roi_led_margin) roi_led_maxx[i] = width;
				else roi_led_maxx[i] += roi_led_margin;
				if (roi_led_minx[i] < roi_led_margin) roi_led_minx[i] = 0;
				else roi_led_minx[i] -= roi_led_margin;
				if (roi_led_maxy[i] > height - roi_led_margin) roi_led_maxy[i] = height;
				else roi_led_maxy[i] += roi_led_margin;
				if (roi_led_miny[i] < roi_led_margin) roi_led_miny[i] = 0;
				else roi_led_miny[i] -= roi_led_margin;
				rois_rand[i].x = roi_led_minx[i];
				rois_rand[i].width = roi_led_maxx[i] - roi_led_minx[i];
				rois_rand[i].y = roi_led_miny[i];
				rois_rand[i].height = roi_led_maxy[i] - roi_led_miny[i];
#ifdef DEBUG_
				cv::rectangle(afterlabel, rois_rand[i], cv::Scalar(255, 255, 255), 1);
#endif // DEBUG_

			}
#ifdef DEBUG_
			QueryPerformanceCounter(&detectend);
			detecttimec = (double)(detectend.QuadPart - detectstartdebug.QuadPart) / freq.QuadPart;
#endif // SHOW_PROCESSING_TIME_


#ifdef DEBUG_
			QueryPerformanceCounter(&detectstartdebug);
#endif // SHOW_PROCESSING_TIME_
			//順番バラバラでもLEDの輝度重心計算
			for (size_t i = 0; i < 4; i++)
			{
				//クラスタ内部に閾値以上の輝点が存在しないときは未検出で終了
				if (ledmass[i] <= 0) {
					processflgs[detectid] = false;
					return 2;
				}
				ledimpos_rand[i][0] = ledmomx[i] / ledmass[i];
				ledimpos_rand[i][1] = ledmomy[i] / ledmass[i];
			}

			//青色から時計回りに緑LEDを当てはめる
			ledimpos[0][0] = ledimpos_rand[blueno][0];
			ledimpos[0][1] = ledimpos_rand[blueno][1];
			rois[0] = rois_rand[blueno];
			ledcog[0] = (ledimpos_rand[0][0] + ledimpos_rand[1][0] + ledimpos_rand[2][0] + ledimpos_rand[3][0]) / 4;
			ledcog[1] = (ledimpos_rand[0][1] + ledimpos_rand[1][1] + ledimpos_rand[2][1] + ledimpos_rand[3][1]) / 4;
			thetamax = 0, thetamin = 10;
			for (size_t i = 0; i < 4; i++)
			{
				if ((int)i == blueno) continue;
				dot = (ledimpos[0][0] - ledcog[0]) * (ledimpos_rand[i][0] - ledcog[0]) + (ledimpos[0][1] - ledcog[1]) * (ledimpos_rand[i][1] - ledcog[1]);
				cross = (ledimpos[0][0] - ledcog[0]) * (ledimpos_rand[i][1] - ledcog[1]) - (ledimpos[0][1] - ledcog[1]) * (ledimpos_rand[i][0] - ledcog[0]);
				theta[i] = atan2(cross, dot);
				if (theta[i] < 0) theta[i] += 2 * M_PI;
				if (thetamax < theta[i])
				{
					thetamax = theta[i];
					thetamaxid = i;
				}
				if (thetamin > theta[i] && theta[i] > 0)
				{
					thetamin = theta[i];
					thetaminid = i;
				}
			}
			for (size_t i = 0; i < 4; i++)
			{
				if (i == thetamaxid)
				{
					ledimpos[3][0] = ledimpos_rand[i][0];
					ledimpos[3][1] = ledimpos_rand[i][1];
					rois[3] = rois_rand[i];
				}
				else if (i == thetaminid)
				{
					ledimpos[1][0] = ledimpos_rand[i][0];
					ledimpos[1][1] = ledimpos_rand[i][1];
					rois[1] = rois_rand[i];
				}
				else if (i == blueno) continue;
				else
				{
					ledimpos[2][0] = ledimpos_rand[i][0];
					ledimpos[2][1] = ledimpos_rand[i][1];
					rois[2] = rois_rand[i];
				}
			}
#ifdef ROI_MODE_
			leddetected = true;
#endif // ROI_MODE_

#ifdef DEBUG_
			QueryPerformanceCounter(&detectend);
			detecttimed = (double)(detectend.QuadPart - detectstartdebug.QuadPart) / freq.QuadPart;
#endif // SHOW_PROCESSING_TIME_

#ifdef DEBUG_
			cout << "DetectLEDMarker() ROI OFF" << endl;
			cout << "DetectLEDMarker() getimgs		:" << detecttimef << endl;
			cout << "DetectLEDMarker() detectV		:" << detecttimea << endl;
			cout << "DetectLEDMarker() clustering	:" << detecttimeb << endl;
			cout << "DetectLEDMarker() calcCoG		:" << detecttimec << endl;
			cout << "DetectLEDMarker() setLED		:" << detecttimed << endl;
#endif // SHOW_PROCESSING_TIME_

		}

		//青と緑両方検出しているとき
		else
		{
			//ON画像がどちらか判定する
			detectimg0_src = detectimg[0].ptr<uint8_t>(0);
			detectimg1_src = detectimg[1].ptr<uint8_t>(0);
			on_img_cnt = 0;
			for (size_t i = 0; i < 4; i++)
			{
				for (size_t k = rois[i].x; k < static_cast<unsigned long long>(rois[i].x) + rois[i].width; k++)
				{
					for (size_t j = rois[i].y; j < static_cast<unsigned long long>(rois[i].y) + rois[i].height; j++)
					{
						if ((int32_t)detectimg0_src[j * width * 3 + k * 3 + 1] > 3 * (int32_t)detectimg1_src[j * width * 3 + k * 3 + 1] && ((int32_t)detectimg0_src[j * width * 3 + k * 3 + 2] > blueLED_min[0] || (int32_t)detectimg0_src[j * width * 3 + k * 3 + 1] > greenLED_min[1]))
						{//2枚の画像で輝度値を比較
							on_img_cnt++;
						}
					}
				}

			}
			if (on_img_cnt > 10) on_img_id = 0;
			else on_img_id = 1;
			detectimg_on_src = detectimg[on_img_id].ptr<uint8_t>(0);

			for (size_t i = 0; i < 4; i++)
			{
				for (size_t k = rois[i].x; k < static_cast<unsigned long long>(rois[i].x) + rois[i].width; k++)
				{
					for (size_t j = rois[i].y; j < static_cast<unsigned long long>(rois[i].y) + rois[i].height; j++)
					{
						if (i == 0)
						{
							if ((int32_t)detectimg_on_src[j * width * 3 + k * 3 + 2] > blueLED_min[0])
							{
								ledmass[i] += (double)detectimg_on_src[j * width * 3 + k * 3 + 2];
								ledmomx[i] += (double)detectimg_on_src[j * width * 3 + k * 3 + 2] * k;
								ledmomy[i] += (double)detectimg_on_src[j * width * 3 + k * 3 + 2] * j;
								//ROIも計算
								if (roi_led_maxx[i] < k) roi_led_maxx[i] = k;
								if (roi_led_minx[i] > k) roi_led_minx[i] = k;
								if (roi_led_maxy[i] < j) roi_led_maxy[i] = j;
								if (roi_led_miny[i] > j) roi_led_miny[i] = j;
							}
						}
						else
						{
							if ((int32_t)detectimg_on_src[j * width * 3 + k * 3 + 1] > greenLED_min[1])
							{
								ledmass[i] += (double)detectimg_on_src[j * width * 3 + k * 3 + 1];
								ledmomx[i] += (double)detectimg_on_src[j * width * 3 + k * 3 + 1] * k;
								ledmomy[i] += (double)detectimg_on_src[j * width * 3 + k * 3 + 1] * j;
								//ROIも計算
								if (roi_led_maxx[i] < k) roi_led_maxx[i] = k;
								if (roi_led_minx[i] > k) roi_led_minx[i] = k;
								if (roi_led_maxy[i] < j) roi_led_maxy[i] = j;
								if (roi_led_miny[i] > j) roi_led_miny[i] = j;
							}
						}
					}
				}
				if (ledmass[i] <= 0)
				{
					//ROI内部に閾値以上の輝点が存在しないときに終了
					leddetected = false;
					processflgs[detectid] = false;
					return 3;
				}
				ledimpos[i][0] = ledmomx[i] / ledmass[i];
				ledimpos[i][1] = ledmomy[i] / ledmass[i];

				if (roi_led_maxx[i] > width - roi_led_margin) roi_led_maxx[i] = width;
				else roi_led_maxx[i] += roi_led_margin;
				if (roi_led_minx[i] < roi_led_margin) roi_led_minx[i] = 0;
				else roi_led_minx[i] -= roi_led_margin;
				if (roi_led_maxy[i] > height - roi_led_margin) roi_led_maxy[i] = height;
				else roi_led_maxy[i] += roi_led_margin;
				if (roi_led_miny[i] < roi_led_margin) roi_led_miny[i] = 0;
				else roi_led_miny[i] -= roi_led_margin;
				rois[i].x = roi_led_minx[i];
				rois[i].width = roi_led_maxx[i] - roi_led_minx[i];
				rois[i].y = roi_led_miny[i];
				rois[i].height = roi_led_maxy[i] - roi_led_miny[i];
			}
#ifdef DEBUG_
			QueryPerformanceCounter(&detectend);
			detecttimea = (double)(detectend.QuadPart - detectstartdebug.QuadPart) / freq.QuadPart;
#endif // SHOW_PROCESSING_TIME_

#ifdef DEBUG_
			cout << "DetectLEDMarker() ROI ON" << endl;
			cout << "DetectLEDMarker() getimgs		:" << detecttimef << endl;
			cout << "DetectLEDMarker() calcCoG		:" << detecttimea << endl;
#endif // SHOW_PROCESSING_TIME_
		}

		//4つのLEDから位置姿勢計算
		///理想ピクセル座標系に変換
#ifdef DEBUG_
		QueryPerformanceCounter(&detectstartdebug);
#endif // SHOW_PROCESSING_TIME_
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
			RTc2m[0][i] = -(lednormdir[i][1] * lednormdir[(i + 2)][2] - lednormdir[i][2] * lednormdir[(i + 2)][1]);
			RTc2m[1][i] = -(lednormdir[i][2] * lednormdir[(i + 2)][0] - lednormdir[i][0] * lednormdir[(i + 2)][2]);
			RTc2m[2][i] = -(lednormdir[i][0] * lednormdir[(i + 2)][1] - lednormdir[i][1] * lednormdir[(i + 2)][0]);
			lambda = 1 / pow(pow(RTc2m[0][i], 2) + pow(RTc2m[1][i], 2) + pow(RTc2m[2][i], 2), 0.5);
			RTc2m[0][i] *= lambda;
			RTc2m[1][i] *= lambda;
			RTc2m[2][i] *= lambda;
		}

		//カメラ-マーカ間の相対姿勢の計算(残りの方向ベクトルを外積で求める)
		RTc2m[0][2] = RTc2m[1][0] * RTc2m[2][1] - RTc2m[2][0] * RTc2m[1][1];
		RTc2m[1][2] = RTc2m[2][0] * RTc2m[0][1] - RTc2m[0][0] * RTc2m[2][1];
		RTc2m[2][2] = RTc2m[0][0] * RTc2m[1][1] - RTc2m[1][0] * RTc2m[0][1];
		lambda = 1 / pow(pow(RTc2m[0][2], 2) + pow(RTc2m[1][2], 2) + pow(RTc2m[2][2], 2), 0.5);
		RTc2m[0][2] *= lambda;
		RTc2m[1][2] *= lambda;
		RTc2m[2][2] *= lambda;

		//ここで，方向ベクトルが画像処理の誤差を乗せて直交しないときに強引に直交する方向ベクトルを計算する
		RTc2m[0][1] = RTc2m[1][2] * RTc2m[2][0] - RTc2m[2][2] * RTc2m[1][0];
		RTc2m[1][1] = RTc2m[2][2] * RTc2m[0][0] - RTc2m[0][2] * RTc2m[2][0];
		RTc2m[2][1] = RTc2m[0][2] * RTc2m[1][0] - RTc2m[1][2] * RTc2m[0][0];
		lambda = 1 / pow(pow(RTc2m[0][1], 2) + pow(RTc2m[1][1], 2) + pow(RTc2m[2][1], 2), 0.5);
		RTc2m[0][1] *= lambda;
		RTc2m[1][1] *= lambda;
		RTc2m[2][1] *= lambda;

		//魚眼モデルと相対姿勢を用いてカメラ-マーカ間の相対位置を計算
		for (size_t i = 0; i < 4; i++)
		{
			Asrc[i * 7 * 3 + i] = RTc2m[0][0] * ledcamdir[i][0] + RTc2m[1][0] * ledcamdir[i][1] + RTc2m[2][0] * ledcamdir[i][2];
			Asrc[i * 7 * 3 + 7 + i] = RTc2m[0][1] * ledcamdir[i][0] + RTc2m[1][1] * ledcamdir[i][1] + RTc2m[2][1] * ledcamdir[i][2];
			Asrc[i * 7 * 3 + 14 + i] = RTc2m[0][2] * ledcamdir[i][0] + RTc2m[1][2] * ledcamdir[i][1] + RTc2m[2][2] * ledcamdir[i][2];
			Asrc[i * 7 * 3 + 4] = 1;
			Asrc[i * 7 * 3 + 12] = 1;
			Asrc[i * 7 * 3 + 20] = 1;
			bsrc[i * 3 + 0] = markerpos[i][0];
			bsrc[i * 3 + 1] = markerpos[i][1];
			bsrc[i * 3 + 2] = 0;
		}
		x = A.inv(cv::DECOMP_SVD) * b;
		RTc2m[3][0] = xsrc[4];
		RTc2m[3][1] = xsrc[5];
		RTc2m[3][2] = xsrc[6];
		//計算された位置に連続性が確認されないときはエラーとする

		//UAVRS2UGVRSの位置姿勢の計算
		RTuavrs2ugvrs = RTugvmk2rs * RTc2m * RTuavrs2hsc;

#ifdef DEBUG_
		QueryPerformanceCounter(&detectend);
		detecttimee = (double)(detectend.QuadPart - detectstartdebug.QuadPart) / freq.QuadPart;
		cout << "DetectLEDMarker() CalcPose		:" << detecttimee << endl;
#endif // 
		processflgs[detectid] = false;
		return 0;
		//位置姿勢も計算できて正常終了
	}
	else
	{
		//入力画像が異常であるときに，強制終了
		leddetected = false;
		return 4;
	}
}

void Read_Reply_toEND(RS232c* robot) {
	char bufreply[256];
	while (true)
	{
		robot->Read_CRLF(bufreply, 256);
		if (bufreply[0] == 'E' || bufreply[0] == 'O') {
			break;
		}
	}
}

void ControlAxisRobot(RS232c* robot, bool* flg) {
	srand(time(NULL));
	int axisspeed = 100;
	int axisposition = 200000;
	int initaxispos = 600;
	char controlcommand[READBUFFERSIZE];
	while (*flg)
	{
		//位置と速度のランダム設定
		if (initaxispos == initaxisend) initaxispos = initaxisstart;
		else if (initaxispos == initaxisstart) initaxispos = initaxisend;
		else initaxispos = initaxisstart;
		axisposition = (initaxispos + rand() % posunits + 1) * 100; //0~100 or 600~700
		axisspeed = (rand() % speedunits + 1) * 10; //10~100で10刻み

		//コマンド送信
		snprintf(controlcommand, READBUFFERSIZE, "@S_17.1=%d\r\n", axisspeed);
		robot->Send(controlcommand);
		memset(controlcommand, '\0', READBUFFERSIZE);
		Read_Reply_toEND(robot);
		snprintf(controlcommand, READBUFFERSIZE, "@START17#P%d.1\r\n", axisposition);
		robot->Send(controlcommand);
		memset(controlcommand, '\0', READBUFFERSIZE);
		Read_Reply_toEND(robot);
	}
}

void wait_QueryPerformance(double finishtime, LARGE_INTEGER freq) {
	LARGE_INTEGER waitstart, waitstop;
	double waittime = 0;
	QueryPerformanceCounter(&waitstart);
	while (waittime < finishtime)
	{
		QueryPerformanceCounter(&waitstop);
		waittime = (double)(waitstop.QuadPart - waitstart.QuadPart) / freq.QuadPart;
	}
}