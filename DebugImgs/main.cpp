#define _USE_MATH_DEFINES

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <cstring>
#include <iostream>
#include <cmath>



#pragma warning(disable:4996)
using namespace std;

const cv::Scalar HSVLED_min(0, 0, 150);
const cv::Scalar HSVLED_max(256, 256, 256);
const cv::Scalar greenLED_min(0, 220, 0);
const cv::Scalar greenLED_max(256, 256, 256);
const cv::Scalar blueLED_min(220, 0, 0);
const cv::Scalar blueLED_max(256, 256, 256);
vector<cv::Point> bluepts, greenpts, Vpts;
cv::Mat diffimg, diffimg_hsv, diffimg_hsv2;
cv::Mat detectgreen, detectblue, detectV;
vector<cv::Mat> detectimg;
vector<cv::Rect> rois, rois_rand;
uint8_t* diffimg_src, * detectimg_on_src, *diffimg_hsv2_src;
cv::Mat labels;
int32_t h;
int greenbluecnt[4][2] = { 0 };
double ledimpos[4][2] = { 0 }, ledidimpos[4][2] = { 0 }, greenimpos[3][2] = { 0 }, ledimpos_rand[4][2] = { 0 };
double bluemass, bluemomx, bluemomy;
double greenmass[3] = { 0 }, greenmomx[3] = { 0 }, greenmomy[3] = { 0 };
double ledmass[4] = { 0 }, ledmomx[4] = { 0 }, ledmomy[4] = { 0 };
int blueno = -1, labelno;
double ledcog[2] = { 0 };
double cross, dot, theta[4];
double thetamax, thetamin, thetamaxid, thetaminid;
const int roi_width = 30;
const int roi_led_margin = 10;


/// カメラパラメータ
const int width = 896;
const int height = 896;
const float fps = 1000.0;
const float exposuretime = 912.0;
const int offsetx = 512;
const int offsety = 92;

const int roi_led_minx_ini[4] = { width, width, width, width}, 
roi_led_maxx_ini[4] = { 0 }, roi_led_miny_ini[4] = { height, height, height, height }, roi_led_maxy_ini[4] = { 0 };
int roi_led_minx[4], roi_led_maxx[4], roi_led_miny[4], roi_led_maxy[4];

int main() {
	string img0_dir = "202101181753_img02.png";
	string img1_dir = "202101181753_img03.png";
	cv::Mat diffimg;

	cv::Mat zero = cv::Mat(896, 896, CV_8UC3, cv::Scalar::all(0));
	//LED位置検出のためのMat vector作成
	for (size_t i = 0; i < 2; i++)
	{
		detectimg.push_back(zero.clone());
	}
	
	diffimg = zero.clone();
	diffimg_src = diffimg.ptr<uint8_t>(0);
	detectimg[0] = cv::imread(img0_dir);
	detectimg[1] = cv::imread(img1_dir);
	detectimg_on_src = detectimg[0].ptr<uint8_t>(0);

	cv::absdiff(detectimg[0], detectimg[1], diffimg);

	cv::cvtColor(diffimg, diffimg_hsv, CV_BGR2HSV);
	//HSV画像でVが閾値以上の座標を検出
	cv::inRange(diffimg_hsv, HSVLED_min, HSVLED_max, detectV);
	cv::findNonZero(detectV, Vpts);

	cv::cvtColor(detectimg[0], diffimg_hsv2, CV_BGR2HSV);
	diffimg_hsv2_src = diffimg_hsv2.ptr<uint8_t>(0);

	for (size_t i = 0; i < 4; i++)
	{
		rois.push_back(cv::Rect(0, 0, width, height));
		rois_rand.push_back(cv::Rect(0, 0, width, height));
	}


	//輝度の高い点群を4か所にクラスタリング
	cv::Mat pts = cv::Mat::zeros(Vpts.size(), 1, CV_32FC2);
	float* ptsptr = pts.ptr<float>(0);
	for (size_t i = 0; i < Vpts.size(); i++)
	{
		ptsptr[i * 2 + 0] = (float)Vpts[i].x;
		ptsptr[i * 2 + 1] = (float)Vpts[i].y;
	}
	cv::kmeans(pts, 4, labels, cvTermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1.0), 1, cv::KMEANS_PP_CENTERS);

	//分類結果の確認
	cv::Mat afterlabel = cv::Mat(896, 896, CV_8UC3, cv::Scalar::all(0));
	uint8_t* afterlabel_src = afterlabel.ptr<uint8_t>(0);
	int32_t* labelptr = labels.ptr<int32_t>(0);
	for (size_t i = 0; i < Vpts.size(); i++)
	{
		//cout << (int32_t)labelptr[i] << endl;
		if ((int32_t)labelptr[i] == 0)
		{
			afterlabel_src[Vpts[i].y * width * 3 + Vpts[i].x * 3] = 60;
			afterlabel_src[Vpts[i].y * width * 3 + Vpts[i].x * 3 + 1] = 20;
			afterlabel_src[Vpts[i].y * width * 3 + Vpts[i].x * 3 + 2] = 220;
		}
		else if ((int32_t)labelptr[i] == 1)
		{
			afterlabel_src[Vpts[i].y * width * 3 + Vpts[i].x * 3] = 0;
			afterlabel_src[Vpts[i].y * width * 3 + Vpts[i].x * 3 + 1] = 215;
			afterlabel_src[Vpts[i].y * width * 3 + Vpts[i].x * 3 + 2] = 255;
		}
		else if ((int32_t)labelptr[i] == 2)
		{
			afterlabel_src[Vpts[i].y * width * 3 + Vpts[i].x * 3] = 127;
			afterlabel_src[Vpts[i].y * width * 3 + Vpts[i].x * 3 + 1] = 255;
			afterlabel_src[Vpts[i].y * width * 3 + Vpts[i].x * 3 + 2] = 0;
		}
		else if ((int32_t)labelptr[i] == 3)
		{
			afterlabel_src[Vpts[i].y * width * 3 + Vpts[i].x * 3] = 255;
			afterlabel_src[Vpts[i].y * width * 3 + Vpts[i].x * 3 + 1] = 191;
			afterlabel_src[Vpts[i].y * width * 3 + Vpts[i].x * 3 + 2] = 0;
		}
	}

	//分類ごとに青緑の個数のカウント
	memset(greenbluecnt, 0, sizeof(int) * 4 * 2);
	for (size_t i = 0; i < Vpts.size(); i++)
	{
		h = (int32_t)diffimg_hsv2_src[Vpts[i].y * width * 3 + Vpts[i].x * 3];
		if ((int32_t)labelptr[i] == 0)
		{
			if (h > 30 && h < 90) greenbluecnt[0][0]++;
			else if (h > 90 && h < 150) greenbluecnt[0][1]++;

		}
		else if ((int32_t)labelptr[i] == 1)
		{
			if (h > 30 && h < 90) greenbluecnt[1][0]++;
			else if (h > 90 && h < 150) greenbluecnt[1][1]++;
		}
		else if ((int32_t)labelptr[i] == 2)
		{
			if (h > 30 && h < 90) greenbluecnt[2][0]++;
			else if (h > 90 && h < 150) greenbluecnt[2][1]++;
		}
		else if ((int32_t)labelptr[i] == 3)
		{
			if (h > 30 && h < 90) greenbluecnt[3][0]++;
			else if (h > 90 && h < 150) greenbluecnt[3][1]++;
		}
	}
	for (size_t i = 0; i < 4; i++)
	{
		cout << greenbluecnt[i][0] << ", " << greenbluecnt[i][1] << endl;
		if (greenbluecnt[i][0] < greenbluecnt[i][1]) blueno = (int)i;
	}
	
	for (size_t i = 0; i < 4; i++)
	{
		ledmass[i] = 0, ledmomx[i] = 0, ledmomy[i] = 0;
	}
	//クラスターごとに輝度重心を計算する
	memcpy(roi_led_minx, roi_led_minx_ini, sizeof(roi_led_minx_ini));
	memcpy(roi_led_maxx, roi_led_maxx_ini, sizeof(roi_led_maxx_ini));
	memcpy(roi_led_miny, roi_led_miny_ini, sizeof(roi_led_miny_ini));
	memcpy(roi_led_maxy, roi_led_maxy_ini, sizeof(roi_led_maxy_ini));
	for (size_t i = 0; i < Vpts.size(); i++)
	{
		labelno = (int)labelptr[i];
		if (labelno == blueno)
		{
			ledmass[labelno] += (double)detectimg_on_src[Vpts[i].y * width * 3 + Vpts[i].x * 3];
			ledmomx[labelno] += (double)detectimg_on_src[Vpts[i].y * width * 3 + Vpts[i].x * 3] * Vpts[i].x;
			ledmomy[labelno] += (double)detectimg_on_src[Vpts[i].y * width * 3 + Vpts[i].x * 3] * Vpts[i].y;
		}
		else
		{
			ledmass[labelno] += (double)detectimg_on_src[Vpts[i].y * width * 3 + Vpts[i].x * 3 + 1];
			ledmomx[labelno] += (double)detectimg_on_src[Vpts[i].y * width * 3 + Vpts[i].x * 3 + 1] * Vpts[i].x;
			ledmomy[labelno] += (double)detectimg_on_src[Vpts[i].y * width * 3 + Vpts[i].x * 3 + 1] * Vpts[i].y;
		}
		//ROIも計算
		if (roi_led_maxx[labelno] < Vpts[i].x) roi_led_maxx[labelno] = Vpts[i].x;
		if (roi_led_minx[labelno] > Vpts[i].x) roi_led_minx[labelno] = Vpts[i].x;
		if (roi_led_maxy[labelno] < Vpts[i].y) roi_led_maxy[labelno] = Vpts[i].y;
		if (roi_led_miny[labelno] > Vpts[i].y) roi_led_miny[labelno] = Vpts[i].y;
	}
	for (size_t i = 0; i < 4; i++)
	{
		if (roi_led_maxx[i] > width - roi_led_margin) roi_led_maxx[i] = width;
		else roi_led_maxx[i] += roi_led_margin;
		if (roi_led_minx[i] < roi_led_margin) roi_led_minx[i] = 0;
		else roi_led_minx[i] -= roi_led_margin;
		if (roi_led_maxy[i] > height- roi_led_margin) roi_led_maxy[i] = height;
		else roi_led_maxy[i] += roi_led_margin;
		if (roi_led_miny[i] < roi_led_margin) roi_led_miny[i] = 0;
		else roi_led_miny[i] -= roi_led_margin;
		rois_rand[i].x = roi_led_minx[i];
		rois_rand[i].width = roi_led_maxx[i] - roi_led_minx[i];
		rois_rand[i].y = roi_led_miny[i];
		rois_rand[i].height = roi_led_maxy[i] - roi_led_miny[i];
	}

	for (size_t i = 0; i < 4; i++)
	{
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
		else if (i==thetaminid)
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


	cout << "LED detected" << endl;

	//一度検出したので，ROIを作成して検出
	for (size_t i = 0; i < 4; i++)
	{
		ledmass[i] = 0, ledmomx[i] = 0, ledmomy[i] = 0;
	}
	memcpy(roi_led_minx, roi_led_minx_ini, sizeof(roi_led_minx_ini));
	memcpy(roi_led_maxx, roi_led_maxx_ini, sizeof(roi_led_maxx_ini));
	memcpy(roi_led_miny, roi_led_miny_ini, sizeof(roi_led_miny_ini));
	memcpy(roi_led_maxy, roi_led_maxy_ini, sizeof(roi_led_maxy_ini));
	for (size_t i = 0; i < 4; i++)
	{
		for (size_t k = rois[i].x; k < static_cast<unsigned long long>(rois[i].x) + rois[i].width; k++)
		{
			for (size_t j = rois[i].y; j < static_cast<unsigned long long>(rois[i].y) + rois[i].height; j++)
			{
				if (i==blueno)
				{
					if ((int32_t)detectimg_on_src[j * width * 3 + k * 3] > blueLED_min[0])
					{
						ledmass[i] += (double)detectimg_on_src[j * width * 3 + k * 3];
						ledmomx[i] += (double)detectimg_on_src[j * width * 3 + k * 3] * k;
						ledmomy[i] += (double)detectimg_on_src[j * width * 3 + k * 3] * j;
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

	return 0;
}