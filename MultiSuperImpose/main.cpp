// includes, system
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <cmath>
#define _USE_MATH_DEFINES

#include <algorithm>            // std::min, std::max
#include <opencv2/opencv.hpp>
#include <iostream>
#include <Windows.h>

#include <librealsense2/rs.hpp>
#include "realsense.h"
#include <thread>
#include <vector>
#include "graphics.h"

//Realsense�Ɋւ���p�����[�^
vector<realsense> rs_devices;
rs2::context context;
const unsigned int colorwidth = 1920;
const unsigned int colorheight = 1080;
const unsigned int colorfps = 30;
const unsigned int depthwidth = 1280;
const unsigned int depthheight = 720;
const unsigned int depthfps = 30;

const int ring_size_realsense = 5;
int getpc_id = 0;
float* texcoords_src;
int update_ringid;

#pragma warning(disable:4996)

//���O�Ɋւ���p�����[�^
struct PointCloud
{
	const rs2::vertex* pc_buffer;
	const rs2::texture_coordinate* texcoords;
	float* pc_ringbuffer;
	float* texcoords_ringbuffer;
	int pc_ringid = 0;
};

//�v���g�^�C�v�錾
void GetPointClouds(realsense* rs, bool* flg, PointCloud* pc);

int main() {
	//�p�����[�^
	bool flg = true;

	//���O������
	cout << "Set PointCloud buffers....";
	PointCloud pc0, pc1;
	pc0.pc_ringbuffer = (float*)malloc(sizeof(float) * ring_size_realsense * vert_cnt * 3);
	pc1.pc_ringbuffer = (float*)malloc(sizeof(float) * ring_size_realsense * vert_cnt * 3);
	pc0.texcoords_ringbuffer = (float*)malloc(sizeof(float) * ring_size_realsense * vert_cnt * 2);

	//PointCloud�擾�p�o�b�t�@
	pc0.pc_buffer = (rs2::vertex*)malloc(sizeof(float) * vert_cnt * 3);
	pc1.pc_buffer = (rs2::vertex*)malloc(sizeof(float) * vert_cnt * 3);
	cout << "OK!" << endl;

	//2��Realsense�̏�����
	cout << "Set RealsenseD435..........";
	const rs2::device_list device_list = context.query_devices();
	for (rs2::device device : device_list)
	{
		realsense rs(device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER), RS2_FORMAT_RGB8, RS2_FORMAT_Z16);
		rs_devices.push_back(rs);
	}
	cout << "OK!" << endl;

	//OpenGL�̏�����
	initGL();

	//�X���b�h�쐬
	thread thr1(GetPointClouds, &rs_devices[0], &flg, &pc0);
	//thread thr2(GetPointClouds, &rs_devices[1], &flg, &pc1);

	//���C�����[�v
	cout << "Main loop start!" << endl;
	while (flg)
	{
		if (GetAsyncKeyState(VK_SPACE) & 0x8000) flg = false;
		getpc_id = pc0.pc_ringid - 1;
		texcoords_src = pc0.texcoords_ringbuffer + getpc_id * vert_cnt * 2;
		drawGL_realsense(pc0.pc_ringbuffer, &pc0.pc_ringid, texcoords_src);
	}

	//OpenGL�̏I��
	finishGL();


	//�X���b�h�폜
	if (thr1.joinable())thr1.join();
	//if (thr2.joinable())thr2.join();

	//���I�������̊J��
	free(pc0.pc_ringbuffer);
	free(pc1.pc_ringbuffer);
	//free(pc0.pc_buffer);
	//free(pc1.pc_buffer);

	return 0;
}

void GetPointClouds(realsense* rs, bool* flg, PointCloud* pc) {
	for (size_t i = 0; i < 30; i++) { rs->update_frame(); }
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
		memcpy((pc->pc_ringbuffer + pc->pc_ringid * vert_cnt * 3), pc->pc_buffer, sizeof(float) * vert_cnt * 3);
		memcpy((pc->texcoords_ringbuffer + pc->pc_ringid * vert_cnt * 2), pc->texcoords, sizeof(float) * vert_cnt * 2);
		
		//�����O�o�b�t�@�̔ԍ����X�V
		pc->pc_ringid = (pc->pc_ringid + 1) % ring_size_realsense;
	}
}