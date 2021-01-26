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
vector<cv::Mat> vecimgs;
vector<cv::Rect> rois, rois_rand;
uint8_t* diffimg_src, * detectimg_on_src, *diffimg_hsv2_src;
uint8_t* detectimg_src, *vecimg_src;
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
const double markeredge = 50;
const double markerpos[4][2] = { {markeredge, markeredge}, {-markeredge, markeredge}, {-markeredge, -markeredge}, {markeredge, -markeredge} };



/// カメラパラメータ
const int width = 896;
const int height = 896;
const float fps = 1000.0;
const float exposuretime = 912.0;
const int offsetx = 512;
const int offsety = 92;
double map_coeff[4], stretch_mat[4], det, distort[2];

const int roi_led_minx_ini[4] = { width, width, width, width}, 
roi_led_maxx_ini[4] = { 0 }, roi_led_miny_ini[4] = { height, height, height, height }, roi_led_maxy_ini[4] = { 0 };
int roi_led_minx[4], roi_led_maxx[4], roi_led_miny[4], roi_led_maxy[4];

int main() {
	string img0_dir = "202101222247_img08.png";
	string img1_dir = "202101222247_img09.png";
	cv::Mat diffimg;

	cv::Mat zero = cv::Mat(896, 896, CV_8UC3, cv::Scalar::all(0));
	//LED位置検出のためのMat vector作成
	for (size_t i = 0; i < 2; i++)
	{
		detectimg.push_back(zero.clone());
		vecimgs.push_back(zero.clone());
	}
	cv::Mat vecimg2 = cv::Mat(896 * 2, 896, CV_8UC3, cv::Scalar::all(0));
	
	diffimg = zero.clone();
	diffimg_src = diffimg.ptr<uint8_t>(0);
	detectimg[0] = cv::imread(img0_dir);
	detectimg[1] = cv::imread(img1_dir);

	//Vector Matの連続性確認
	detectimg_src = detectimg[0].ptr<uint8_t>(0);
	vecimg_src = vecimg2.ptr<uint8_t>(0);
	memcpy(vecimg_src, detectimg_src, width * height * 3);
	memcpy(vecimg_src + width * height * 3, detectimg[1].data, width * height * 3);

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

	//4津のLEDから位置姿勢計算
	//レーザCalibrationの結果の呼び出し
	FILE* fcam;
	fcam = fopen("202012300316_fisheyeparam.csv", "r");
	for (size_t i = 0; i < 4; i++) { fscanf(fcam, "%lf,", &map_coeff[i]); }
	for (size_t i = 0; i < 4; i++) { fscanf(fcam, "%lf,", &stretch_mat[i]); }
	swap(stretch_mat[1], stretch_mat[2]);
	for (size_t i = 0; i < 2; i++) { fscanf(fcam, "%lf,", &distort[i]); }
	fclose(fcam);
	det = 1 / (stretch_mat[0] - stretch_mat[1] * stretch_mat[2]);

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
		Rm2c[0][i] = -(lednormdir[i][1] * lednormdir[(i + 2)][2] - lednormdir[i][2] * lednormdir[(i + 2)][1]);
		Rm2c[1][i] = -(lednormdir[i][2] * lednormdir[(i + 2)][0] - lednormdir[i][0] * lednormdir[(i + 2)][2]);
		Rm2c[2][i] = -(lednormdir[i][0] * lednormdir[(i + 2)][1] - lednormdir[i][1] * lednormdir[(i + 2)][0]);
		lambda = 1 / pow(pow(Rm2c[0][i], 2) + pow(Rm2c[1][i], 2) + pow(Rm2c[2][i], 2), 0.5);
		Rm2c[0][i] *= lambda;
		Rm2c[1][i] *= lambda;
		Rm2c[2][i] *= lambda;
	}

	//カメラ-マーカ間の相対姿勢の計算(残りの方向ベクトルを外積で求める)
	Rm2c[0][2] = Rm2c[1][0] * Rm2c[2][1] - Rm2c[2][0] * Rm2c[1][1];
	Rm2c[1][2] = Rm2c[2][0] * Rm2c[0][1] - Rm2c[0][0] * Rm2c[2][1];
	Rm2c[2][2] = Rm2c[0][0] * Rm2c[1][1] - Rm2c[1][0] * Rm2c[0][1];
	lambda = 1 / pow(pow(Rm2c[0][2], 2) + pow(Rm2c[1][2], 2) + pow(Rm2c[2][2], 2), 0.5);
	Rm2c[0][2] *= lambda;
	Rm2c[1][2] *= lambda;
	Rm2c[2][2] *= lambda;

	//ここで，方向ベクトルが画像処理の誤差を乗せて直交しないときに強引に直交する方向ベクトルを計算する
	Rm2c[0][1] = Rm2c[1][2] * Rm2c[2][0] - Rm2c[2][2] * Rm2c[1][0];
	Rm2c[1][1] = Rm2c[2][2] * Rm2c[0][0] - Rm2c[0][2] * Rm2c[2][0];
	Rm2c[2][1] = Rm2c[0][2] * Rm2c[1][0] - Rm2c[1][2] * Rm2c[0][0];
	lambda = 1 / pow(pow(Rm2c[0][1], 2) + pow(Rm2c[1][1], 2) + pow(Rm2c[2][1], 2), 0.5);
	Rm2c[0][1] *= lambda;
	Rm2c[1][1] *= lambda;
	Rm2c[2][1] *= lambda;

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

	return 0;
}