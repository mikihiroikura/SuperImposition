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

//�o�͂Ɋւ���p�����[�^
float* gl_pc_src[realsense_cnt];
float* gl_texcoord_src[realsense_cnt];
rs2::frame* gl_tex_src[realsense_cnt];

//���ԂɊւ���ϐ�
LARGE_INTEGER start, stop, freq;
LARGE_INTEGER glstart, glstop;
LARGE_INTEGER takestart, takeend;
LARGE_INTEGER showstart, showend;
double taketime = 0, showtime = 0;
double timer = 0, gltimer = 0;

//���O�Ɋւ���p�����[�^
struct PointCloud
{
	const rs2::vertex* pc_buffer;
	const rs2::texture_coordinate* texcoords;
	float* pc_ringbuffer;
	float* texcoords_ringbuffer;
	rs2::frame colorframe_buffer[ring_size_realsense];
	int pc_ringid = 0;
};
PointCloud* pcs_ptr[realsense_cnt];

void GetPointClouds(realsense* rs, bool* flg, PointCloud* pc);
void TakePicture(kayacoaxpress* cam, bool* flg);
void ShowAllLogs(bool* flg);

int main() {
	//�p�����[�^
	bool flg = true;
	if (!QueryPerformanceFrequency(&freq)) { return 0; }// �P�ʏK��

	//�J�����̏�����
	kayacoaxpress cam;
	cam.connect(0);

	//�J�����p�����[�^�̐ݒ�
	cout << "Set Camera Params..." << endl;
	cam.setParam(paramTypeKAYACoaXpress::AcquisitionMode::TriggerMode, 1); //�g���K�[���[�h�ŋN��
	cam.setParam(paramTypeCamera::paramInt::WIDTH, width);
	cam.setParam(paramTypeCamera::paramInt::HEIGHT, height);
	cam.setParam(paramTypeKAYACoaXpress::paramInt::OffsetX, offsetx);
	cam.setParam(paramTypeKAYACoaXpress::paramInt::OffsetY, offsety);
	cam.setParam(paramTypeCamera::paramFloat::FPS, fps);
	cam.setParam(paramTypeKAYACoaXpress::paramFloat::ExposureTime, exposuretime);
	cam.setParam(paramTypeKAYACoaXpress::Gain::x2);
	cam.setParam(paramTypeKAYACoaXpress::CaptureType::BayerGRGrab);
	cam.parameter_all_print();

	//�摜�擾�p�̃����O�o�b�t�@�̍쐬
	full = cv::Mat(cam.getParam(paramTypeCamera::paramInt::HEIGHT), cam.getParam(paramTypeCamera::paramInt::WIDTH), CV_8UC3, cv::Scalar::all(255));
	zero = cv::Mat(cam.getParam(paramTypeCamera::paramInt::HEIGHT), cam.getParam(paramTypeCamera::paramInt::WIDTH), CV_8UC3, cv::Scalar::all(0));
	zeromulti = cv::Mat(cam.getParam(paramTypeCamera::paramInt::HEIGHT) * multicnt, cam.getParam(paramTypeCamera::paramInt::WIDTH), CV_8UC3, cv::Scalar::all(0));
	cout << "Set Mat Cycle Buffer..." << endl;
	for (size_t i = 0; i < ringbuffersize; i++)
	{
		in_imgs_on.push_back(zero.clone());
		in_imgs_off.push_back(zero.clone());
		in_imgs.push_back(zeromulti.clone());
		processflgs.push_back(false);
	}

	//���O������
	cout << "Set PointCloud buffers....";
	vector<PointCloud> pcs;

	for (size_t i = 0; i < realsense_cnt; i++)
	{
		PointCloud pc;
		pc.pc_ringbuffer = (float*)malloc(sizeof(float) * ring_size_realsense * vert_cnt * 3);
		pc.texcoords_ringbuffer = (float*)malloc(sizeof(float) * ring_size_realsense * vert_cnt * 2);
		pc.pc_buffer = (rs2::vertex*)malloc(sizeof(float) * vert_cnt * 3);
		pc.texcoords = (rs2::texture_coordinate*)malloc(sizeof(float) * vert_cnt * 2);
		pcs.push_back(pc);
	}
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

	//OpenGL�̏�����
	initGL();

	//�J�����N��
	cout << "Camera Start!" << endl;
	cam.start();

	//�X���b�h�쐬
	vector<thread> GetPointsThread;
	for (size_t i = 0; i < realsense_cnt; i++)
	{
		GetPointsThread.push_back(thread(GetPointClouds, &rs_devices[i], &flg, &pcs[i]));
	}
	thread TakePictureThread(TakePicture, &cam, &flg);
	thread ShowLogsThread(ShowAllLogs, &flg);

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
		QueryPerformanceCounter(&glstart);
		if (GetAsyncKeyState(VK_ESCAPE) & 0x8000) flg = false;

		//�Ăяo���_�Q�̃|�C���^
		for (size_t i = 0; i < realsense_cnt; i++)
		{
			getpc_id = pcs[i].pc_ringid - 1;
			if (getpc_id < 0) getpc_id += ring_size_realsense;
			gl_pc_src[i] = pcs[i].pc_ringbuffer + (unsigned long long)getpc_id * 3 * vert_cnt;
			gl_texcoord_src[i] = pcs[i].texcoords_ringbuffer + (unsigned long long)getpc_id * 2 * vert_cnt;
			gl_tex_src[i] = pcs[i].colorframe_buffer + getpc_id;
		}
		drawGL_realsense(gl_pc_src, gl_texcoord_src, gl_tex_src);
		QueryPerformanceCounter(&glstop);
		gltimer = (double)(glstop.QuadPart - glstart.QuadPart) / freq.QuadPart;
		cout << "OpenGL time[s]: " << gltimer << endl;

	}

	//OpenGL�̏I��
	finishGL();


	//�X���b�h�폜
	for (size_t i = 0; i < GetPointsThread.size(); i++)
	{
		if (GetPointsThread[i].joinable())GetPointsThread[i].join();
	}
	if (TakePictureThread.joinable())TakePictureThread.join();
	if (ShowLogsThread.joinable())ShowLogsThread.join();

	//���I�������̊J��
	/*for (size_t i = 0; i < pcs.size(); i++)
	{
		free(pcs[i].pc_ringbuffer);
		free(pcs[i].texcoords_ringbuffer);
	}*/

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

//�摜�̓_�Q�S�Ă�\��
void ShowAllLogs(bool* flg) {
	while (*flg)
	{
		QueryPerformanceCounter(&showstart);

		//OpenCV�ŉ摜�\��
		cv::imshow("img", in_imgs[(in_imgs_saveid - 2 + ringbuffersize) % ringbuffersize]);
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
