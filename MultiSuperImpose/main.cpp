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

//Realsenseに関するパラメータ
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

//ログに関するパラメータ
struct PointCloud
{
	const rs2::vertex* pc_buffer;
	const rs2::texture_coordinate* texcoords;
	float* pc_ringbuffer;
	float* texcoords_ringbuffer;
	int pc_ringid = 0;
};

//プロトタイプ宣言
void GetPointClouds(realsense* rs, bool* flg, PointCloud* pc);

int main() {
	//パラメータ
	bool flg = true;

	//ログ初期化
	cout << "Set PointCloud buffers....";
	PointCloud pc0, pc1;
	pc0.pc_ringbuffer = (float*)malloc(sizeof(float) * ring_size_realsense * vert_cnt * 3);
	pc1.pc_ringbuffer = (float*)malloc(sizeof(float) * ring_size_realsense * vert_cnt * 3);
	pc0.texcoords_ringbuffer = (float*)malloc(sizeof(float) * ring_size_realsense * vert_cnt * 2);

	//PointCloud取得用バッファ
	pc0.pc_buffer = (rs2::vertex*)malloc(sizeof(float) * vert_cnt * 3);
	pc1.pc_buffer = (rs2::vertex*)malloc(sizeof(float) * vert_cnt * 3);
	cout << "OK!" << endl;

	//2つのRealsenseの初期化
	cout << "Set RealsenseD435..........";
	const rs2::device_list device_list = context.query_devices();
	for (rs2::device device : device_list)
	{
		realsense rs(device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER), RS2_FORMAT_RGB8, RS2_FORMAT_Z16);
		rs_devices.push_back(rs);
	}
	cout << "OK!" << endl;

	//OpenGLの初期化
	initGL();

	//スレッド作成
	thread thr1(GetPointClouds, &rs_devices[0], &flg, &pc0);
	//thread thr2(GetPointClouds, &rs_devices[1], &flg, &pc1);

	//メインループ
	cout << "Main loop start!" << endl;
	while (flg)
	{
		if (GetAsyncKeyState(VK_SPACE) & 0x8000) flg = false;
		getpc_id = pc0.pc_ringid - 1;
		texcoords_src = pc0.texcoords_ringbuffer + getpc_id * vert_cnt * 2;
		drawGL_realsense(pc0.pc_ringbuffer, &pc0.pc_ringid, texcoords_src);
	}

	//OpenGLの終了
	finishGL();


	//スレッド削除
	if (thr1.joinable())thr1.join();
	//if (thr2.joinable())thr2.join();

	//動的メモリの開放
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
		//点群計算
		rs->update_frame();
		rs->update_color();
		rs->update_depth();
		rs->calc_pointcloud();
		pc->pc_buffer = rs->points.get_vertices();
		pc->texcoords = rs->points.get_texture_coordinates();
		//取得点群をリングバッファに保存
		memcpy((pc->pc_ringbuffer + pc->pc_ringid * vert_cnt * 3), pc->pc_buffer, sizeof(float) * vert_cnt * 3);
		memcpy((pc->texcoords_ringbuffer + pc->pc_ringid * vert_cnt * 2), pc->texcoords, sizeof(float) * vert_cnt * 2);
		
		//リングバッファの番号を更新
		pc->pc_ringid = (pc->pc_ringid + 1) % ring_size_realsense;
	}
}