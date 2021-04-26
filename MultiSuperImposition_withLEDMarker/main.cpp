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

#ifdef _DEBUG
#define LIB_EXT "d.lib"
#else
#define LIB_EXT ".lib"
#endif


#pragma comment(lib,"KAYACoaXpressLib" LIB_EXT)
#pragma warning(disable:4996)
using namespace std;


//�J�����p�����[�^
const int width = 896;
const int height = 896;
const float fps = 1000.0;
const float exposuretime = 912.0;
const int offsetx = 512;
const int offsety = 92;
double map_coeff[4], stretch_mat[4], det, distort[4];

/// �摜�Ɋւ���p�����[�^
const int ringbuffersize = 10;
vector<cv::Mat> in_imgs_on, in_imgs_off, in_imgs;
vector<int> in_imgs_on_nums;
vector<bool> processflgs;
cv::Mat zero, full, zeromulti;
int takepicid, in_imgs_saveid;
const int multicnt = 2;
uint8_t* in_img_multi_src, * detectimg_multi_src;

//Realsense�Ɋւ���p�����[�^
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

//�o�͂Ɋւ���p�����[�^
float* gl_pc_src[realsense_cnt];
float* gl_texcoord_src[realsense_cnt];
rs2::frame* gl_tex_src[realsense_cnt];

//���ԂɊւ���ϐ�
LARGE_INTEGER start, stop, freq;
LARGE_INTEGER glstart, glstop;
LARGE_INTEGER takestart, takeend;
LARGE_INTEGER showstart, showend;
LARGE_INTEGER logstart, logend;
LARGE_INTEGER detectstart, detectstartdebug, detectend;
double taketime = 0, showtime = 0, logtime = 0;
double timer = 0, gltimer = 0;
double detecttimea = 0, detecttimeb = 0, detecttimec = 0, detecttimed = 0, detecttimee = 0, detecttimef = 0, detecttime = 0;

//�}�[�J���o�Ɋւ���p�����[�^
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
glm::mat4 RTm2c = glm::mat4(1.0), RTc2m = glm::mat4(1.0), RTuavrs2ugvrs = glm::mat4(1.0);
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
const double markeredge = 235 / 2 * 0.001;//�P�ʂ�m
const double markerpos[4][2] = { {markeredge, markeredge}, {-markeredge, markeredge}, {-markeredge, -markeredge}, {markeredge, -markeredge} };
int detectresult = -1;



//���O�Ɋւ���p�����[�^
const int timeout = 20;
const int log_img_fps = 40;
const int log_img_finish_cnt = log_img_fps * timeout + 100;
const int log_pose_finish_cnt = fps * timeout + 100;
long long log_img_cnt = 0, log_lsm_cnt = 0;
uint8_t* save_img_on_src;
bool saveimgflg = false, savestart = false;
struct Logs
{
	vector<cv::Mat> in_imgs_log;
	cv::Mat gl_img;
	vector<cv::Mat> gl_imgs_log;
	cv::Mat* in_imgs_log_ptr;
	cv::Mat* gl_imgs_log_ptr;
};

//�v���g�^�C�v�錾
void GetPointClouds(realsense* rs, bool* flg, PointCloud* pc);
void TakePicture(kayacoaxpress* cam, bool* flg);
void ShowAllLogs(bool* flg, PointCloud** pc_src, Logs* logs);
int DetectLEDMarker();

using namespace std;

#define GETPOINTSREALSENSE_THREAD_
#define GETRELPOSE_THREAD_
#define SHOW_PROCESSING_TIME_
#define SHOW_IMGS_THREAD_
#define SHOW_OPENGL_THREAD_
//#define DEBUG_
#define ROI_MODE_

#define SAVE_IMGS_
#define SAVE_HSC2MK_POSE_

int main() {
	//�p�����[�^
	bool flg = true;
	if (!QueryPerformanceFrequency(&freq)) { return 0; }// �P�ʏK��

	//�J�����̏�����
	kayacoaxpress cam;
	cam.connect(1);

	//�J�����p�����[�^�̐ݒ�
	cout << "Set Camera Params..." << endl;
	cam.setParam(paramTypeKAYACoaXpress::AcquisitionMode::TriggerMode, 1); //�g���K�[���[�h�ŋN��
	cam.setParam(paramTypeCamera::paramInt::WIDTH, width);
	cam.setParam(paramTypeCamera::paramInt::HEIGHT, height);
	cam.setParam(paramTypeKAYACoaXpress::paramInt::OffsetX, offsetx);
	cam.setParam(paramTypeKAYACoaXpress::paramInt::OffsetY, offsety);
	cam.setParam(paramTypeCamera::paramFloat::FPS, fps);
	cam.setParam(paramTypeKAYACoaXpress::paramFloat::ExposureTime, exposuretime);
	cam.setParam(paramTypeKAYACoaXpress::Gain::x1);
	cam.setParam(paramTypeKAYACoaXpress::CaptureType::BayerGRGrab);
	cam.parameter_all_print();

	//�摜�擾�p�̃����O�o�b�t�@�̍쐬
	full = cv::Mat(cam.getParam(paramTypeCamera::paramInt::HEIGHT), cam.getParam(paramTypeCamera::paramInt::WIDTH), CV_8UC3, cv::Scalar::all(255));
	zero = cv::Mat(cam.getParam(paramTypeCamera::paramInt::HEIGHT), cam.getParam(paramTypeCamera::paramInt::WIDTH), CV_8UC3, cv::Scalar::all(0));
	zeromulti = cv::Mat(cam.getParam(paramTypeCamera::paramInt::HEIGHT) * multicnt, cam.getParam(paramTypeCamera::paramInt::WIDTH), CV_8UC3, cv::Scalar::all(0));
	cout << "Set Mat Cycle Buffer..." << endl;
	for (size_t i = 0; i < ringbuffersize; i++)
	{
		in_imgs.push_back(zeromulti.clone());
		in_imgs_on_nums.push_back(-1);
		processflgs.push_back(false);
	}

	//���[�UCalibration�̌��ʂ̌Ăяo��
	FILE* fcam;
	fcam = fopen("202104191534_fisheyeparam.csv", "r");
	for (size_t i = 0; i < 4; i++) { fscanf(fcam, "%lf,", &map_coeff[i]); }
	for (size_t i = 0; i < 4; i++) { fscanf(fcam, "%lf,", &stretch_mat[i]); }
	swap(stretch_mat[1], stretch_mat[2]);
	for (size_t i = 0; i < 2; i++) { fscanf(fcam, "%lf,", &distort[i]); }
	fclose(fcam);
	det = 1 / (stretch_mat[0] - stretch_mat[1] * stretch_mat[2]);

	//PoseCalibration���ʂ̌Ăяo��
	FILE* fpose;
	fpose = fopen("202104231718_poseparam.csv", "r");
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
		RTuavrs2hsc[3][i] /= 1000;//�P�ʂ�m
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
		RTugvmk2rs[3][i] /= 1000;//�P�ʂ�m
	}

	//�P�_�ۑ��p�s��̍쐬
	ptscand = cv::Mat::zeros(width * height, 1, CV_32FC2);
	ptscand_ptr = ptscand.ptr<float>(0);

	//ROI�̐ݒ�
	for (size_t i = 0; i < 4; i++)
	{
		rois.push_back(cv::Rect(0, 0, width, height));
		rois_rand.push_back(cv::Rect(0, 0, width, height));
	}

	//LED�ʒu���o�̂��߂�Mat vector�쐬
	for (size_t i = 0; i < 2; i++)
	{
		detectimg.push_back(zero.clone());
	}
	diffimg = zero.clone();
	diffimg_src = diffimg.ptr<uint8_t>(0);

	//�v�Z�����ʒu�p���̕ۑ��p�����O�o�b�t�@�̍쐬
	RTuavrs2ugvrs_buffer = (glm::mat4*)malloc(sizeof(glm::mat4) * ringbuffersize);
	for (size_t i = 0; i < ringbuffersize; i++)
	{
		memcpy((RTuavrs2ugvrs_buffer + i), &RTuavrs2ugvrs, sizeof(glm::mat4));
	}

	//PointCloud������
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

	//2��Realsense�̏�����
	cout << "Set RealsenseD435..........";
	const rs2::device_list device_list = context.query_devices();
	for (rs2::device device : device_list)
	{
		realsense rs(device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER), RS2_FORMAT_RGB8, colorwidth, colorheight, colorfps, RS2_FORMAT_Z16, depthwidth, depthheight, depthfps);
		rs_devices.push_back(rs);
	}
	cout << "OK!" << endl;

	//OpenGL�o�͂̉摜�ƈʒu�p���v�Z�̉摜�ۑ��o�b�t�@�̍쐬
	Logs logs;
#ifdef SAVE_IMGS_
	//�擾�摜���i�[����Vector�̍쐬
	std::cout << "Set Img Vector for logs....................";
	logs.gl_img = cv::Mat(window_height, window_width, CV_8UC3, cv::Scalar::all(0));
	for (size_t i = 0; i < log_img_finish_cnt; i++) { logs.gl_imgs_log.push_back(logs.gl_img.clone()); }
	for (size_t i = 0; i < log_img_finish_cnt; i++) { logs.in_imgs_log.push_back(zero.clone()); }
	logs.in_imgs_log_ptr = logs.in_imgs_log.data();
	logs.gl_imgs_log_ptr = logs.gl_imgs_log.data();
	cout << "OK!" << endl;
#endif // SAVE_IMGS_

	//�J�����N��
	cout << "Camera Start!" << endl;
	cam.start();

	//�X���b�h�쐬
#ifdef GETPOINTSREALSENSE_THREAD_
	vector<thread> GetPointsThread;
	for (size_t i = 0; i < realsense_cnt; i++)
	{
		GetPointsThread.push_back(thread(GetPointClouds, &rs_devices[i], &flg, &pcs[i]));
	}
#endif // GETPOINTREALSENSE_THREAD_
	thread TakePictureThread(TakePicture, &cam, &flg);
#ifdef SHOW_IMGS_THREAD_
	thread ShowLogsThread(ShowAllLogs, &flg, pcs_src, &logs);
#endif // SHOW_	


	//���C�����[�v
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

		if (detectresult == 0)
		{//�����Ɉʒu�p�����茋�ʂ𔽉f������v�Z������
			memcpy((RTuavrs2ugvrs_buffer + RTuavrs2ugvrs_bufferid), &RTuavrs2ugvrs, sizeof(glm::mat4));
			RTuavrs2ugvrs_bufferid = (RTuavrs2ugvrs_bufferid + 1) % ringbuffersize;
		}
		QueryPerformanceCounter(&detectend);
		detecttime = (double)(detectend.QuadPart - detectstart.QuadPart) / freq.QuadPart;
		while (detecttime < 0.002)
		{
			QueryPerformanceCounter(&detectend);
			detecttime = (double)(detectend.QuadPart - detectstart.QuadPart) / freq.QuadPart;
		}
#ifdef SHOW_PROCESSING_TIME_
		cout << "DetectLEDMarker() result: " << detectresult << endl;
		std::cout << "DetectLEDMarker() time: " << detecttime << endl;
#endif // SHOW_PROCESSING_TIME_
	}

	//�X���b�h�폜
#ifdef GETPOINTSREALSENSE_THREAD_
	for (size_t i = 0; i < GetPointsThread.size(); i++)
	{
		if (GetPointsThread[i].joinable())GetPointsThread[i].join();
	}
#endif // GETPOINTSREALSENSE_THREAD_
	if (TakePictureThread.joinable())TakePictureThread.join();
#ifdef SHOW_IMGS_THREAD_
	if (ShowLogsThread.joinable())ShowLogsThread.join();
#endif // SHOW_IMGS_THREAD_

	//���O�ۑ��̂��߂̏���
	FILE* fr;
	time_t timer;
	struct tm now;
	timer = time(NULL);
	localtime_s(&now, &timer);


	//�擾�����摜�̕ۑ�
#ifdef SAVE_IMGS_
	if (log_img_cnt > 0)
	{
		std::cout << "Saving imgs..." << endl;
		//HSC�̉摜�ۑ�
		char picdir[256];
		struct stat statBuf;
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
		for (int i = 0; i < log_img_cnt; i++)
		{
			sprintf(picturename, "%s%05d.png", picsubname, i);//png�t���k
			cv::imwrite(picturename, logs.in_imgs_log[i]);
		}
		//OpenGL�̉摜�ۑ�
		strftime(picdir, 256, "D:/Github_output/SuperImposition/MultiSuperImposition_withLEDMarker/results/%y%m%d/%H%M%S/GL", &now);
		if (stat(picdir, &statBuf) != 0) {
			if (_mkdir(picdir) != 0) { return 0; }
		}
		strftime(picsubname, 256, "D:/Github_output/SuperImposition/MultiSuperImposition_withLEDMarker/results/%y%m%d/%H%M%S/GL/frame", &now);
		for (int i = 0; i < log_img_cnt; i++)
		{
			sprintf(picturename, "%s%05d.png", picsubname, i);//png�t���k
			cv::flip(logs.gl_imgs_log[i], logs.gl_imgs_log[i], 0);
			cv::imwrite(picturename, logs.gl_imgs_log[i]);
		}
		std::cout << "Imgs finished!" << endl;
	}
	
#endif // SAVE_IMGS_


	return 0;
}

//�摜���i�[����
void TakePicture(kayacoaxpress* cam, bool* flg) {
	while (*flg)
	{
		QueryPerformanceCounter(&takestart);
		takepicid = in_imgs_saveid % ringbuffersize;
		in_img_multi_src = in_imgs[takepicid].ptr<uint8_t>(0);

		cam->captureFrame(in_img_multi_src, multicnt);

		in_imgs_saveid = (in_imgs_saveid + 1) % ringbuffersize;
		processflgs[takepicid] = true;
		QueryPerformanceCounter(&takeend);
		taketime = (double)(takeend.QuadPart - takestart.QuadPart) / freq.QuadPart;
		while (taketime < 0.002)
		{
			QueryPerformanceCounter(&takeend);
			taketime = (double)(takeend.QuadPart - takestart.QuadPart) / freq.QuadPart;
		}
#ifdef SHOW_PROCESSING_TIME_
		std::cout << "TakePicture() time: " << taketime << endl;
#endif // SHOW_PROCESSING_TIME_
	}
}

//�摜�̓_�Q�S�Ă�\��
void ShowAllLogs(bool* flg, PointCloud** pc_src, Logs *logs) {
	//OpenGL�̏�����
	initGL();

	QueryPerformanceCounter(&showstart);
	while (showtime < 1.5)
	{
		QueryPerformanceCounter(&showend);
		showtime = (double)(showend.QuadPart - showstart.QuadPart) / freq.QuadPart;
	}
	while (*flg)
	{
		QueryPerformanceCounter(&showstart);

		//OpenCV�ŉ摜�\��
		cv::imshow("img", in_imgs[(in_imgs_saveid - 2 + ringbuffersize) % ringbuffersize]);
		int key = cv::waitKey(1);
		if (key == 'q') *flg = false;
		if (key == 's' && !savestart) {
			saveimgflg = true;
			QueryPerformanceCounter(&logstart);
			savestart = true;
		}

#ifdef SAVE_IMGS_
		if (saveimgflg)
		{
			//LED ON�摜�̕ۑ�
			save_img_on_src = in_imgs[(in_imgs_saveid - 2 + ringbuffersize) % ringbuffersize].ptr<uint8_t>(0);
			if (in_imgs_on_nums[(in_imgs_saveid - 2 + ringbuffersize) % ringbuffersize] == 0)
			{
				memcpy((logs->in_imgs_log_ptr + log_img_cnt)->data, save_img_on_src, height * width * 3);
			}
			else
			{
				memcpy((logs->in_imgs_log_ptr + log_img_cnt)->data, save_img_on_src + height * width * 3, height * width * 3);
			}
			//OpenGL�\���̉摜�ۑ�
			saveImgCV((logs->gl_imgs_log_ptr + log_img_cnt)->data);

			log_img_cnt++;
			if (log_img_cnt > log_img_finish_cnt) *flg = false;

			//���Ԍv��
			QueryPerformanceCounter(&logend);
			logtime = (double)(logend.QuadPart - logstart.QuadPart) / freq.QuadPart;
			if (logtime > timeout) *flg = false;
		}
		

#endif // SAVE_IMGS_

#ifdef SHOW_OPENGL_THREAD_
		//OpenGL��2���Realsense�̓_�Q�o��
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

	//OpenGL�̏I��
	finishGL();
}

void ShowHSCLogs() {

}

//RealSense����_�Q��摜�̎擾
void GetPointClouds(realsense* rs, bool* flg, PointCloud* pc) {
	while (*flg)
	{
		//�_�Q�v�Z
		rs->update_frame();
		rs->update_color();
		rs->update_depth();
		rs->calc_pointcloud();
		pc->pc_buffer = rs->points.get_vertices();
		pc->texcoords = rs->points.get_texture_coordinates();
		//�擾�_�Q�������O�o�b�t�@�ɕۑ�
		memcpy((pc->pc_ringbuffer + (unsigned long long)pc->pc_ringid * vert_cnt * 3), pc->pc_buffer, sizeof(float) * vert_cnt * 3);
		memcpy((pc->texcoords_ringbuffer + (unsigned long long)pc->pc_ringid * vert_cnt * 2), pc->texcoords, sizeof(float) * vert_cnt * 2);
		pc->colorframe_buffer[pc->pc_ringid] = rs->colorframe;

		//�����O�o�b�t�@�̔ԍ����X�V
		pc->pc_ringid = (pc->pc_ringid + 1) % ring_size_realsense;
	}
}

int DetectLEDMarker() {
#ifdef DEBUG_
	QueryPerformanceCounter(&detectstartdebug);
#endif // 

	//�摜�̊i�[
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
	//LED�������o�̎��́C�摜�S�̂�T������
	if (detectimg[0].data != NULL && detectimg[1].data != NULL && (int)detectimg[0].data[0] != 255 && (int)detectimg[1].data[0] != 255 && (int)detectimg[0].data[0] != 0 && (int)detectimg[1].data[0] != 0)
	{
		//�N���X�^�[���ƂɋP�x�d�S���v�Z����
		for (size_t i = 0; i < 4; i++)
		{
			ledmass[i] = 0, ledmomx[i] = 0, ledmomy[i] = 0;
		}
		memcpy(roi_led_minx, roi_led_minx_ini, sizeof(roi_led_minx_ini));
		memcpy(roi_led_maxx, roi_led_maxx_ini, sizeof(roi_led_maxx_ini));
		memcpy(roi_led_miny, roi_led_miny_ini, sizeof(roi_led_miny_ini));
		memcpy(roi_led_maxy, roi_led_maxy_ini, sizeof(roi_led_maxy_ini));

		//4�S�Ă�LED�����o���Ă��Ȃ���
		if (!(leddetected))
		{
			//�����摜�̐���
			cv::absdiff(detectimg[0], detectimg[1], diffimg);
			//HSV��V��臒l����
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
			//�����ō����摜����P�_��������Ȃ��Ƃ��̗�O����������
			if (ptscnt <= 0) return 6;
#ifdef DEBUG_
			QueryPerformanceCounter(&detectend);
			detecttimea = (double)(detectend.QuadPart - detectstartdebug.QuadPart) / freq.QuadPart;
#endif // SHOW_PROCESSING_TIME_

#ifdef DEBUG_
			QueryPerformanceCounter(&detectstartdebug);
#endif // SHOW_PROCESSING_TIME_
			//�P�x�̍����_�Q��4�����ɃN���X�^�����O
			pts = ptscand(cv::Rect(0, 0, 1, ptscnt));
			cv::kmeans(pts, 4, labels, cvTermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1.0), 1, cv::KMEANS_PP_CENTERS, centers);
			center_src = centers.ptr<float>(0);
			//�N���X�^�Ԃ̋�����臒l�ȉ��Ȃ�Ζ����o�Ɣ���
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
			//�f�o�b�O:���ތ��ʂ̊m�F
			afterlabel = cv::Mat(896, 896, CV_8UC3, cv::Scalar::all(0));
			afterlabel_src = afterlabel.ptr<uint8_t>(0);
			labelptr_debug = labels.ptr<int32_t>(0);
			for (size_t i = 0; i < ptscnt; i++)
			{
				//cout << (int32_t)labelptr[i] << endl;
				if ((int32_t)labelptr_debug[i] == 0)
				{//��
					afterlabel_src[(int)ptscand_ptr[i * 2 + 1] * width * 3 + (int)ptscand_ptr[i * 2 + 0] * 3] = 60;
					afterlabel_src[(int)ptscand_ptr[i * 2 + 1] * width * 3 + (int)ptscand_ptr[i * 2 + 0] * 3 + 1] = 20;
					afterlabel_src[(int)ptscand_ptr[i * 2 + 1] * width * 3 + (int)ptscand_ptr[i * 2 + 0] * 3 + 2] = 220;
				}
				else if ((int32_t)labelptr_debug[i] == 1)
				{//���F
					afterlabel_src[(int)ptscand_ptr[i * 2 + 1] * width * 3 + (int)ptscand_ptr[i * 2 + 0] * 3] = 0;
					afterlabel_src[(int)ptscand_ptr[i * 2 + 1] * width * 3 + (int)ptscand_ptr[i * 2 + 0] * 3 + 1] = 215;
					afterlabel_src[(int)ptscand_ptr[i * 2 + 1] * width * 3 + (int)ptscand_ptr[i * 2 + 0] * 3 + 2] = 255;
				}
				else if ((int32_t)labelptr_debug[i] == 2)
				{//��
					afterlabel_src[(int)ptscand_ptr[i * 2 + 1] * width * 3 + (int)ptscand_ptr[i * 2 + 0] * 3] = 127;
					afterlabel_src[(int)ptscand_ptr[i * 2 + 1] * width * 3 + (int)ptscand_ptr[i * 2 + 0] * 3 + 1] = 255;
					afterlabel_src[(int)ptscand_ptr[i * 2 + 1] * width * 3 + (int)ptscand_ptr[i * 2 + 0] * 3 + 2] = 0;
				}
				else if ((int32_t)labelptr_debug[i] == 3)
				{//��
					afterlabel_src[(int)ptscand_ptr[i * 2 + 1] * width * 3 + (int)ptscand_ptr[i * 2 + 0] * 3] = 255;
					afterlabel_src[(int)ptscand_ptr[i * 2 + 1] * width * 3 + (int)ptscand_ptr[i * 2 + 0] * 3 + 1] = 191;
					afterlabel_src[(int)ptscand_ptr[i * 2 + 1] * width * 3 + (int)ptscand_ptr[i * 2 + 0] * 3 + 2] = 0;
				}
			}
#endif // DEBUG

#ifdef DEBUG_
			QueryPerformanceCounter(&detectstartdebug);
#endif // SHOW_PROCESSING_TIME_
			//ON�摜���ǂ��炩���肷��
			detectimg0_src = detectimg[0].ptr<uint8_t>(0);
			detectimg1_src = detectimg[1].ptr<uint8_t>(0);
			on_img_cnt = 0;
			for (size_t i = 0; i < ptscnt; i++)
			{
				if ((int32_t)detectimg0_src[(int)ptscand_ptr[i * 2 + 1] * width * 3 + (int)ptscand_ptr[i * 2 + 0] * 3] > (int32_t)detectimg1_src[(int)ptscand_ptr[i * 2 + 1] * width * 3 + (int)ptscand_ptr[i * 2 + 0] * 3])
				{//2���̉摜�ŋP�x�l���r
					on_img_cnt++;
				}
			}
			if (on_img_cnt > ptscnt / 2) on_img_id = 0;
			else on_img_id = 1;
			detectimg_on_src = detectimg[on_img_id].ptr<uint8_t>(0);
			in_imgs_on_nums[detectid] = on_img_id;

			//���ނ��Ƃɐ΂̌��̃J�E���g
			blueno = -1;
			cv::cvtColor(detectimg[on_img_id], detectimg_on_hsv, CV_BGR2HSV);
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

			//�N���X�^���ƂɋP�x�d�S�̌v�Z
			for (size_t i = 0; i < ptscnt; i++)
			{
				labelno = (int)labelptr[i];
				dist = hypot(center_src[labelno * 2 + 0] - (int)ptscand_ptr[i * 2 + 0], center_src[labelno * 2 + 1] - (int)ptscand_ptr[i * 2 + 1]);
				if (dist < dist_cluster_thr)
				{
					if (labelno == blueno)
					{
						if ((int32_t)detectimg_on_src[(int)ptscand_ptr[i * 2 + 1] * width * 3 + (int)ptscand_ptr[i * 2 + 0] * 3] > blueLED_min(0))
						{//On�摜�̐�臒l�͂����ƍ���
							ledmass[labelno] += (double)detectimg_on_src[(int)ptscand_ptr[i * 2 + 1] * width * 3 + (int)ptscand_ptr[i * 2 + 0] * 3];
							ledmomx[labelno] += (double)detectimg_on_src[(int)ptscand_ptr[i * 2 + 1] * width * 3 + (int)ptscand_ptr[i * 2 + 0] * 3] * (int)ptscand_ptr[i * 2 + 0];
							ledmomy[labelno] += (double)detectimg_on_src[(int)ptscand_ptr[i * 2 + 1] * width * 3 + (int)ptscand_ptr[i * 2 + 0] * 3] * (int)ptscand_ptr[i * 2 + 1];
							//ROI���v�Z
							if (roi_led_maxx[labelno] < (int)ptscand_ptr[i * 2 + 0]) roi_led_maxx[labelno] = (int)ptscand_ptr[i * 2 + 0];
							if (roi_led_minx[labelno] > (int)ptscand_ptr[i * 2 + 0]) roi_led_minx[labelno] = (int)ptscand_ptr[i * 2 + 0];
							if (roi_led_maxy[labelno] < (int)ptscand_ptr[i * 2 + 1]) roi_led_maxy[labelno] = (int)ptscand_ptr[i * 2 + 1];
							if (roi_led_miny[labelno] > (int)ptscand_ptr[i * 2 + 1]) roi_led_miny[labelno] = (int)ptscand_ptr[i * 2 + 1];
						}
					}
					else
					{//On�摜�̗΂�臒l�͂����ƍ���
						if ((int32_t)detectimg_on_src[(int)ptscand_ptr[i * 2 + 1] * width * 3 + (int)ptscand_ptr[i * 2 + 0] * 3 + 1] > greenLED_min(1))
						{
							ledmass[labelno] += (double)detectimg_on_src[(int)ptscand_ptr[i * 2 + 1] * width * 3 + (int)ptscand_ptr[i * 2 + 0] * 3 + 1];
							ledmomx[labelno] += (double)detectimg_on_src[(int)ptscand_ptr[i * 2 + 1] * width * 3 + (int)ptscand_ptr[i * 2 + 0] * 3 + 1] * (int)ptscand_ptr[i * 2 + 0];
							ledmomy[labelno] += (double)detectimg_on_src[(int)ptscand_ptr[i * 2 + 1] * width * 3 + (int)ptscand_ptr[i * 2 + 0] * 3 + 1] * (int)ptscand_ptr[i * 2 + 1];
							//ROI���v�Z
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
			//���ԃo���o���ł�LED�̋P�x�d�S�v�Z
			for (size_t i = 0; i < 4; i++)
			{
				//�N���X�^������臒l�ȏ�̋P�_�����݂��Ȃ��Ƃ��͖����o�ŏI��
				if (ledmass[i] <= 0) {
					processflgs[detectid] = false;
					return 2;
				}
				ledimpos_rand[i][0] = ledmomx[i] / ledmass[i];
				ledimpos_rand[i][1] = ledmomy[i] / ledmass[i];
			}

			//�F���玞�v���ɗ�LED�𓖂Ă͂߂�
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

		//�ƗΗ������o���Ă���Ƃ�
		else
		{
			//ON�摜���ǂ��炩���肷��
			detectimg0_src = detectimg[0].ptr<uint8_t>(0);
			detectimg1_src = detectimg[1].ptr<uint8_t>(0);
			on_img_cnt = 0;
			for (size_t i = 0; i < 4; i++)
			{
				for (size_t k = rois[i].x; k < static_cast<unsigned long long>(rois[i].x) + rois[i].width; k++)
				{
					for (size_t j = rois[i].y; j < static_cast<unsigned long long>(rois[i].y) + rois[i].height; j++)
					{
						if ((int32_t)detectimg0_src[j * width * 3 + k * 3] > 3 * (int32_t)detectimg1_src[j * width * 3 + k * 3])
						{//2���̉摜�ŋP�x�l���r
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
						if (i == blueno)
						{
							if ((int32_t)detectimg_on_src[j * width * 3 + k * 3] > blueLED_min[0])
							{
								ledmass[i] += (double)detectimg_on_src[j * width * 3 + k * 3];
								ledmomx[i] += (double)detectimg_on_src[j * width * 3 + k * 3] * k;
								ledmomy[i] += (double)detectimg_on_src[j * width * 3 + k * 3] * j;
								//ROI���v�Z
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
								//ROI���v�Z
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
					//ROI������臒l�ȏ�̋P�_�����݂��Ȃ��Ƃ��ɏI��
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

		//4��LED����ʒu�p���v�Z
		///���z�s�N�Z�����W�n�ɕϊ�
#ifdef DEBUG_
		QueryPerformanceCounter(&detectstartdebug);
#endif // SHOW_PROCESSING_TIME_
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
			RTm2c[i][0] = -(lednormdir[i][1] * lednormdir[(i + 2)][2] - lednormdir[i][2] * lednormdir[(i + 2)][1]);
			RTm2c[i][1] = -(lednormdir[i][2] * lednormdir[(i + 2)][0] - lednormdir[i][0] * lednormdir[(i + 2)][2]);
			RTm2c[i][2] = -(lednormdir[i][0] * lednormdir[(i + 2)][1] - lednormdir[i][1] * lednormdir[(i + 2)][0]);
			lambda = 1 / pow(pow(RTm2c[i][0], 2) + pow(RTm2c[i][1], 2) + pow(RTm2c[i][2], 2), 0.5);
			RTm2c[i][0] *= lambda;
			RTm2c[i][1] *= lambda;
			RTm2c[i][2] *= lambda;
		}

		//�J����-�}�[�J�Ԃ̑��Ύp���̌v�Z(�c��̕����x�N�g�����O�ςŋ��߂�)
		RTm2c[2][0] = RTm2c[0][1] * RTm2c[1][2] - RTm2c[0][2] * RTm2c[1][1];
		RTm2c[2][1] = RTm2c[0][2] * RTm2c[1][0] - RTm2c[0][0] * RTm2c[1][2];
		RTm2c[2][2] = RTm2c[0][0] * RTm2c[1][1] - RTm2c[0][1] * RTm2c[1][0];
		lambda = 1 / pow(pow(RTm2c[2][0], 2) + pow(RTm2c[2][1], 2) + pow(RTm2c[2][2], 2), 0.5);
		RTm2c[2][0] *= lambda;
		RTm2c[2][1] *= lambda;
		RTm2c[2][2] *= lambda;

		//�����ŁC�����x�N�g�����摜�����̌덷���悹�Ē������Ȃ��Ƃ��ɋ����ɒ�����������x�N�g�����v�Z����
		RTm2c[1][0] = RTm2c[2][1] * RTm2c[0][2] - RTm2c[2][2] * RTm2c[0][1];
		RTm2c[1][1] = RTm2c[2][2] * RTm2c[0][0] - RTm2c[2][0] * RTm2c[0][2];
		RTm2c[1][2] = RTm2c[2][0] * RTm2c[0][1] - RTm2c[2][1] * RTm2c[0][0];
		lambda = 1 / pow(pow(RTm2c[1][0], 2) + pow(RTm2c[1][1], 2) + pow(RTm2c[1][2], 2), 0.5);
		RTm2c[1][0] *= lambda;
		RTm2c[1][1] *= lambda;
		RTm2c[1][2] *= lambda;

		//���Ⴢ�f���Ƒ��Ύp����p���ăJ����-�}�[�J�Ԃ̑��Έʒu���v�Z
		for (size_t i = 0; i < 4; i++)
		{
			Asrc[i * 7 * 3 + i] = ledcamdir[i][0];
			Asrc[i * 7 * 3 + 7 + i] = ledcamdir[i][1];
			Asrc[i * 7 * 3 + 14 + i] = ledcamdir[i][2];
			Asrc[i * 7 * 3 + 4] = -1;
			Asrc[i * 7 * 3 + 12] = -1;
			Asrc[i * 7 * 3 + 20] = -1;
			bsrc[i * 3 + 0] = RTm2c[0][0] * markerpos[i][0] + RTm2c[1][0] * markerpos[i][1];
			bsrc[i * 3 + 1] = RTm2c[0][1] * markerpos[i][0] + RTm2c[1][1] * markerpos[i][1];
			bsrc[i * 3 + 2] = RTm2c[0][2] * markerpos[i][0] + RTm2c[1][2] * markerpos[i][1];
		}
		x = A.inv(cv::DECOMP_SVD) * b;
		RTm2c[3][0] = xsrc[4];
		RTm2c[3][1] = xsrc[5];
		RTm2c[3][2] = xsrc[6];
		//�v�Z���ꂽ�ʒu�ɘA�������m�F����Ȃ��Ƃ��̓G���[�Ƃ���

		//UAVRS2UGVRS�̈ʒu�p���̌v�Z
		RTc2m = glm::inverse(RTm2c);

		RTuavrs2ugvrs = RTugvmk2rs * RTc2m * RTuavrs2hsc;

#ifdef DEBUG_
		QueryPerformanceCounter(&detectend);
		detecttimee = (double)(detectend.QuadPart - detectstartdebug.QuadPart) / freq.QuadPart;
		cout << "DetectLEDMarker() CalcPose		:" << detecttimee << endl;
#endif // 
		processflgs[detectid] = false;
		return 0;
		//�ʒu�p�����v�Z�ł��Đ���I��
	}
	else
	{
		//���͉摜���ُ�ł���Ƃ��ɁC�����I��
		leddetected = false;
		return 4;
	}
}
