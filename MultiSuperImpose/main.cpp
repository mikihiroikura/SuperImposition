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


#include <thread>
#include <vector>
#include "graphics.h"

//Realsenseに関するパラメータ
vector<realsense> rs_devices;
rs2::context context;

const int ring_size_realsense = 5;
int getpc_id = 0;
float* texcoords_src;
int update_ringid;

//出力に関するパらめーた
float* gl_pc_src[realsense_cnt];
float* gl_texcoord_src[realsense_cnt];
rs2::frame* gl_tex_src[realsense_cnt];

//時間に関する変数
LARGE_INTEGER start, stop, freq;
LARGE_INTEGER glstart, glstop;
double timer = 0, gltimer = 0;

#pragma warning(disable:4996)

//ログに関するパラメータ
struct PointCloud
{
	const rs2::vertex* pc_buffer;
	const rs2::texture_coordinate* texcoords;
	float* pc_ringbuffer;
	float* texcoords_ringbuffer;
	rs2::frame colorframe_buffer[ring_size_realsense];
	int pc_ringid = 0;
};

//プロトタイプ宣言
void GetPointClouds(realsense* rs, bool* flg, PointCloud* pc);

int main() {
	//パラメータ
	bool flg = true;
	if (!QueryPerformanceFrequency(&freq)) { return 0; }// 単位習得

	//ログ初期化
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

	//2つのRealsenseの初期化
	cout << "Set RealsenseD435..........";
	const rs2::device_list device_list = context.query_devices();
	for (rs2::device device : device_list)
	{
		realsense rs(device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER), RS2_FORMAT_RGB8, colorwidth, colorheight, colorfps, RS2_FORMAT_Z16, depthwidth, depthheight, depthfps);
		rs_devices.push_back(rs);
	}
	cout << "OK!" << endl;

	//OpenGLの初期化
	initGL();

	//スレッド作成
	vector<thread> GetPointsThread;
	for (size_t i = 0; i < realsense_cnt; i++)
	{
		GetPointsThread.push_back(thread(GetPointClouds, &rs_devices[i], &flg, &pcs[i]));
	}

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
		QueryPerformanceCounter(&glstart);
		if (GetAsyncKeyState(VK_SPACE) & 0x8000) flg = false;
		
		//呼び出す点群のポインタ
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

	//OpenGLの終了
	finishGL();


	//スレッド削除
	for (size_t i = 0; i < GetPointsThread.size(); i++)
	{
		if (GetPointsThread[i].joinable())GetPointsThread[i].join();
	}

	//動的メモリの開放
	/*for (size_t i = 0; i < pcs.size(); i++)
	{
		free(pcs[i].pc_ringbuffer);
		free(pcs[i].texcoords_ringbuffer);
	}*/

	return 0;
}

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