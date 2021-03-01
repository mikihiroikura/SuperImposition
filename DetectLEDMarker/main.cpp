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

#ifdef _DEBUG
#define LIB_EXT "d.lib"
#else
#define LIB_EXT ".lib"
#endif


#pragma comment(lib,"KAYACoaXpressLib" LIB_EXT)
#pragma warning(disable:4996)
using namespace std;

//グローバル変数
/// カメラパラメータ
const int width = 896;
const int height = 896;
const float fps = 1000.0;
const float exposuretime = 912.0;
const int offsetx = 480;
const int offsety = 92;
double map_coeff[4], stretch_mat[4], det, distort[4];
/// 画像に関するパラメータ
const int ringbuffersize = 10;
vector<cv::Mat> in_imgs_on, in_imgs_off, in_imgs;
vector<bool> processflgs;
cv::Mat zero, full, zeromulti;
int takepicid, in_imgs_saveid;
const int multicnt = 2;
uint8_t* in_img_multi_src, *detectimg_multi_src;
/// 時間計測に関するパラメータ
LARGE_INTEGER takestart, takeend, freq;
LARGE_INTEGER showstart, showend;
LARGE_INTEGER detectstart, detectend;
double taketime = 0, showtime = 0;
double detecttimea = 0, detecttimeb = 0, detecttimec = 0, detecttimed = 0, detecttimee = 0;
/// マーカ検出のためのパラメータ
int detectid = 0;
vector<cv::Mat> detectimg;
cv::Mat diffimg ,diffimg_hsv, detectimg_on_hsv;
uint8_t* diffimg_src, *detectimg0_src, *detectimg1_src, *detectimg_on_src, *detectimghsv_on_src, *afterlabel_src;;
cv::Mat detectgreen, detectblue, detectV;
const cv::Scalar greenLED_min(0, 150, 0);
const cv::Scalar greenLED_max(256, 256, 256);
const cv::Scalar blueLED_min(150, 0, 0);
const cv::Scalar blueLED_max(256, 256, 256);
const cv::Scalar HSVLED_min(0, 0, 100);
const cv::Scalar HSVLED_max(256, 256, 256);
vector<cv::Point> Vpts;
double ledimpos[4][2] = { 0 }, ledidimpos[4][2] = { 0 }, ledimpos_rand[4][2] = { 0 };;
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
const double markeredge = 235/2;
const double markerpos[4][2] = { {markeredge, markeredge}, {-markeredge, markeredge}, {-markeredge, -markeredge}, {markeredge, -markeredge} };
double cross, dot, theta[3], absmax;
bool leddetected = false;
cv::Mat pts, labels, centers, afterlabel;
float* center_src;
int32_t* labelptr;
int greenbluecnt[4][2] = { 0 };
int on_img_id;
int on_img_cnt;
int32_t h_on;
int blueno = -1, labelno;
const int roi_led_minx_ini[4] = { width, width, width, width },
roi_led_maxx_ini[4] = { 0 }, roi_led_miny_ini[4] = { height, height, height, height }, roi_led_maxy_ini[4] = { 0 };
int roi_led_minx[4], roi_led_maxx[4], roi_led_miny[4], roi_led_maxy[4];
double ledmass[4] = { 0 }, ledmomx[4] = { 0 }, ledmomy[4] = { 0 };
vector<cv::Rect> rois, rois_rand;
const int roi_led_margin = 10;
double thetamax, thetamin, thetamaxid, thetaminid;
double dist, dist_cluster_thr = 20, dist_centers_thr = 10;
int detectled_result;
float *ptsptr;
int32_t *labelptr_debug;


#define SHOW_PROCESSING_TIME_
#define SHOW_IMGS_OPENGL_
#define DEBUG_
#define ROI_MODE_

///プロトタイプ宣言
void TakePicture(kayacoaxpress* cam, bool* flg);
int DetectLEDMarker();
void ShowAllLogs(bool* flg);


int main() {
	bool flg = true;

	QueryPerformanceFrequency(&freq);

	kayacoaxpress cam;
	cam.connect(0);

	//パラメータの設定
	cout << "Set Camera Params..." << endl;
	cam.setParam(paramTypeKAYACoaXpress::AcquisitionMode::TriggerMode, 1); //トリガーモードで起動
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
	fcam = fopen("202101070034_fisheyeparam_cam0.csv", "r");
	for (size_t i = 0; i < 4; i++) { fscanf(fcam, "%lf,", &map_coeff[i]); }
	for (size_t i = 0; i < 4; i++) { fscanf(fcam, "%lf,", &stretch_mat[i]); }
	swap(stretch_mat[1], stretch_mat[2]);
	for (size_t i = 0; i < 2; i++) { fscanf(fcam, "%lf,", &distort[i]); }
	fclose(fcam);
	det = 1 / (stretch_mat[0] - stretch_mat[1] * stretch_mat[2]);

	full = cv::Mat(cam.getParam(paramTypeCamera::paramInt::HEIGHT), cam.getParam(paramTypeCamera::paramInt::WIDTH), CV_8UC3, cv::Scalar::all(255));
	zero = cv::Mat(cam.getParam(paramTypeCamera::paramInt::HEIGHT), cam.getParam(paramTypeCamera::paramInt::WIDTH), CV_8UC3, cv::Scalar::all(0));
	zeromulti = cv::Mat(cam.getParam(paramTypeCamera::paramInt::HEIGHT) * multicnt, cam.getParam(paramTypeCamera::paramInt::WIDTH), CV_8UC3, cv::Scalar::all(0));

	//ROIの設定
	for (size_t i = 0; i < 4; i++)
	{
		rois.push_back(cv::Rect(0, 0, width, height));
		rois_rand.push_back(cv::Rect(0, 0, width, height));
	}

	//ring Bufferの生成
	cout << "Set Mat ring Buffer..." << endl;
	for (size_t i = 0; i < ringbuffersize; i++)
	{
		in_imgs_on.push_back(zero.clone());
		in_imgs_off.push_back(zero.clone());
		in_imgs.push_back(zeromulti.clone());
		processflgs.push_back(false);
	}

	//LED位置検出のためのMat vector作成
	for (size_t i = 0; i < 2; i++)
	{
		detectimg.push_back(zero.clone());
	}
	diffimg = zero.clone();
	diffimg_src = diffimg.ptr<uint8_t>(0);

	//カメラ起動
	cout << "Camera Start!" << endl;
	cam.start();

	//Threadの作成
	thread thr1(TakePicture, &cam, &flg);
#ifdef SHOW_IMGS_OPENGL_
	thread thr2(ShowAllLogs, &flg);
#endif // SHOW_IMGS_OPENGL_


	while (flg)
	{
		detectled_result = DetectLEDMarker();
		cout << "Detect LED marker result: " << detectled_result << endl;
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

//画像とOpenGLの点群全てを表示
void ShowAllLogs(bool* flg) {
	while (*flg)
	{
		QueryPerformanceCounter(&showstart);

		//OpenCVで画像表示
		cv::imshow("img", in_imgs[(in_imgs_saveid - 2 + ringbuffersize) % ringbuffersize]);
		int key = cv::waitKey(1);
		if (key == 'q') *flg = false;

#ifdef SAVE_IMGS_
		memcpy((imglog + log_img_cnt)->data, in_imgs[(in_imgs_saveid - 2 + ringbuffersize) % ringbuffersize].data, height * width * 3);
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

int DetectLEDMarker() {
	QueryPerformanceCounter(&detectstart);
	//画像の格納
	detectid = (in_imgs_saveid - 1 + ringbuffersize) % ringbuffersize;
	detectimg_multi_src = in_imgs[detectid].ptr<uint8_t>(0);
	if (processflgs[detectid])
	{
		memcpy(detectimg[0].data, detectimg_multi_src, height * width * 3);
		memcpy(detectimg[1].data, detectimg_multi_src + height * width * 3, height * width * 3);
	}
	//LEDが未検出の時は，画像全体を探索する
	if (detectimg[0].data!=NULL && detectimg[1].data != NULL && (int)detectimg[0].data[0]!=255 && (int)detectimg[1].data[0] != 255 && (int)detectimg[0].data[0] != 0 && (int)detectimg[1].data[0] != 0)
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
			//差分画像の生成&HSV画像への変換
			cv::absdiff(detectimg[0], detectimg[1], diffimg);
			cv::cvtColor(diffimg, diffimg_hsv, CV_BGR2HSV);
			//HSV画像でVが閾値以上の座標を検出
			cv::inRange(diffimg_hsv, HSVLED_min, HSVLED_max, detectV);//差分画像なので閾値が小さい
			cv::findNonZero(detectV, Vpts);
			/*diffimg_src = diffimg.ptr<uint8_t>(0);
			Vpts.clear();
			for (size_t i = 0; i < width; i++)
			{
				for (size_t j = 0; j < height; j++)
				{
					if ((uint8_t)diffimg_src[j * width * 3 + i * 3] > HSVLED_min(2) || (uint8_t)diffimg_src[j * width * 3 + i * 3 + 1] > HSVLED_min(2) || (uint8_t)diffimg_src[j * width * 3 + i * 3 + 2] > HSVLED_min(2))
					{
						Vpts.push_back(cv::Point(i, j));
					}
				}
			}*/
#ifdef SHOW_PROCESSING_TIME_
			QueryPerformanceCounter(&detectend);
			detecttimea = (double)(detectend.QuadPart - detectstart.QuadPart) / freq.QuadPart;
#endif // SHOW_PROCESSING_TIME_

#ifdef SHOW_PROCESSING_TIME_
			QueryPerformanceCounter(&detectstart);
#endif // SHOW_PROCESSING_TIME_
			//輝度の高い点群を4か所にクラスタリング
			pts = cv::Mat::zeros(Vpts.size(), 1, CV_32FC2);
			ptsptr = pts.ptr<float>(0);
			for (size_t i = 0; i < Vpts.size(); i++)
			{
				ptsptr[i * 2 + 0] = (float)Vpts[i].x;
				ptsptr[i * 2 + 1] = (float)Vpts[i].y;
			}
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
#ifdef SHOW_PROCESSING_TIME_
			QueryPerformanceCounter(&detectend);
			detecttimeb = (double)(detectend.QuadPart - detectstart.QuadPart) / freq.QuadPart;
#endif // SHOW_PROCESSING_TIME_



#ifdef DEBUG_
			//デバッグ:分類結果の確認
			afterlabel = cv::Mat(896, 896, CV_8UC3, cv::Scalar::all(0));
			afterlabel_src = afterlabel.ptr<uint8_t>(0);
			labelptr_debug = labels.ptr<int32_t>(0);
			for (size_t i = 0; i < Vpts.size(); i++)
			{
				//cout << (int32_t)labelptr[i] << endl;
				if ((int32_t)labelptr_debug[i] == 0)
				{//赤
					afterlabel_src[Vpts[i].y * width * 3 + Vpts[i].x * 3] = 60;
					afterlabel_src[Vpts[i].y * width * 3 + Vpts[i].x * 3 + 1] = 20;
					afterlabel_src[Vpts[i].y * width * 3 + Vpts[i].x * 3 + 2] = 220;
				}
				else if ((int32_t)labelptr_debug[i] == 1)
				{//黄色
					afterlabel_src[Vpts[i].y * width * 3 + Vpts[i].x * 3] = 0;
					afterlabel_src[Vpts[i].y * width * 3 + Vpts[i].x * 3 + 1] = 215;
					afterlabel_src[Vpts[i].y * width * 3 + Vpts[i].x * 3 + 2] = 255;
				}
				else if ((int32_t)labelptr_debug[i] == 2)
				{//緑
					afterlabel_src[Vpts[i].y * width * 3 + Vpts[i].x * 3] = 127;
					afterlabel_src[Vpts[i].y * width * 3 + Vpts[i].x * 3 + 1] = 255;
					afterlabel_src[Vpts[i].y * width * 3 + Vpts[i].x * 3 + 2] = 0;
				}
				else if ((int32_t)labelptr_debug[i] == 3)
				{//青
					afterlabel_src[Vpts[i].y * width * 3 + Vpts[i].x * 3] = 255;
					afterlabel_src[Vpts[i].y * width * 3 + Vpts[i].x * 3 + 1] = 191;
					afterlabel_src[Vpts[i].y * width * 3 + Vpts[i].x * 3 + 2] = 0;
				}
			}
#endif // DEBUG

#ifdef SHOW_PROCESSING_TIME_
			QueryPerformanceCounter(&detectstart);
#endif // SHOW_PROCESSING_TIME_
			//ON画像がどちらか判定する
			detectimg0_src = detectimg[0].ptr<uint8_t>(0);
			detectimg1_src = detectimg[1].ptr<uint8_t>(0);
			on_img_cnt = 0;
			for (size_t i = 0; i < Vpts.size(); i++)
			{
				if ((int32_t)detectimg0_src[Vpts[0].y * width * 3 + Vpts[0].x * 3] > (int32_t)detectimg1_src[Vpts[0].y * width * 3 + Vpts[0].x * 3])
				{//2枚の画像で輝度値を比較
					on_img_cnt++;
				}
			}
			if (on_img_cnt > Vpts.size() / 2) on_img_id = 0;
			else on_img_id = 1;
			detectimg_on_src = detectimg[on_img_id].ptr<uint8_t>(0);

			//分類ごとに青緑の個数のカウント
			blueno = -1;
			cv::cvtColor(detectimg[on_img_id], detectimg_on_hsv, CV_BGR2HSV);
			detectimghsv_on_src = detectimg_on_hsv.ptr<uint8_t>(0);
			labelptr = labels.ptr<int32_t>(0);
			memset(greenbluecnt, 0, sizeof(int) * 4 * 2);
			for (size_t i = 0; i < Vpts.size(); i++)
			{
				h_on = (int32_t)detectimghsv_on_src[Vpts[i].y * width * 3 + Vpts[i].x * 3];
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
				if (greenbluecnt[i][0] < greenbluecnt[i][1]) blueno = (int)i;
			}
			if (blueno == -1) return 5;

			//クラスタごとに輝度重心の計算
			for (size_t i = 0; i < Vpts.size(); i++)
			{
				labelno = (int)labelptr[i];
				dist = hypot(center_src[labelno * 2 + 0] - Vpts[i].x, center_src[labelno * 2 + 1] - Vpts[i].y);
				if (dist < dist_cluster_thr)
				{
					if (labelno == blueno)
					{
						if ((int32_t)detectimg_on_src[Vpts[i].y * width * 3 + Vpts[i].x * 3] > blueLED_min(0))
						{//On画像の青の閾値はもっと高い
							ledmass[labelno] += (double)detectimg_on_src[Vpts[i].y * width * 3 + Vpts[i].x * 3];
							ledmomx[labelno] += (double)detectimg_on_src[Vpts[i].y * width * 3 + Vpts[i].x * 3] * Vpts[i].x;
							ledmomy[labelno] += (double)detectimg_on_src[Vpts[i].y * width * 3 + Vpts[i].x * 3] * Vpts[i].y;
							//ROIも計算
							if (roi_led_maxx[labelno] < Vpts[i].x) roi_led_maxx[labelno] = Vpts[i].x;
							if (roi_led_minx[labelno] > Vpts[i].x) roi_led_minx[labelno] = Vpts[i].x;
							if (roi_led_maxy[labelno] < Vpts[i].y) roi_led_maxy[labelno] = Vpts[i].y;
							if (roi_led_miny[labelno] > Vpts[i].y) roi_led_miny[labelno] = Vpts[i].y;
						}
					}
					else
					{//On画像の緑の閾値はもっと高い
						if ((int32_t)detectimg_on_src[Vpts[i].y * width * 3 + Vpts[i].x * 3 + 1] > greenLED_min(1))
						{
							ledmass[labelno] += (double)detectimg_on_src[Vpts[i].y * width * 3 + Vpts[i].x * 3 + 1];
							ledmomx[labelno] += (double)detectimg_on_src[Vpts[i].y * width * 3 + Vpts[i].x * 3 + 1] * Vpts[i].x;
							ledmomy[labelno] += (double)detectimg_on_src[Vpts[i].y * width * 3 + Vpts[i].x * 3 + 1] * Vpts[i].y;
							//ROIも計算
							if (roi_led_maxx[labelno] < Vpts[i].x) roi_led_maxx[labelno] = Vpts[i].x;
							if (roi_led_minx[labelno] > Vpts[i].x) roi_led_minx[labelno] = Vpts[i].x;
							if (roi_led_maxy[labelno] < Vpts[i].y) roi_led_maxy[labelno] = Vpts[i].y;
							if (roi_led_miny[labelno] > Vpts[i].y) roi_led_miny[labelno] = Vpts[i].y;
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
#ifdef SHOW_PROCESSING_TIME_
			QueryPerformanceCounter(&detectend);
			detecttimec = (double)(detectend.QuadPart - detectstart.QuadPart) / freq.QuadPart;
#endif // SHOW_PROCESSING_TIME_


#ifdef SHOW_PROCESSING_TIME_
			QueryPerformanceCounter(&detectstart);
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

#ifdef SHOW_PROCESSING_TIME_
			QueryPerformanceCounter(&detectend);
			detecttimed = (double)(detectend.QuadPart - detectstart.QuadPart) / freq.QuadPart;
#endif // SHOW_PROCESSING_TIME_

#ifdef SHOW_PROCESSING_TIME_
			cout << "DetectLEDMarker() ROI OFF" << endl;
			cout << "DetectLEDMarker() detectV		:" << detecttimea << endl;
			cout << "DetectLEDMarker() clustering	:" << detecttimeb << endl;
			cout << "DetectLEDMarker() calcCoG		:" << detecttimec << endl;
			cout << "DetectLEDMarker() setLED		:" << detecttimed << endl;
#endif // SHOW_PROCESSING_TIME_

		}

		//青と緑両方検出しているとき
		else
		{
#ifdef SHOW_PROCESSING_TIME_
			QueryPerformanceCounter(&detectstart);
#endif // SHOW_PROCESSING_TIME_
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
						if ((int32_t)detectimg0_src[j * width * 3 + k * 3] > 3 * (int32_t)detectimg1_src[j * width * 3 + k * 3])
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
						if (i == blueno)
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
#ifdef SHOW_PROCESSING_TIME_
			QueryPerformanceCounter(&detectend);
			detecttimea = (double)(detectend.QuadPart - detectstart.QuadPart) / freq.QuadPart;
#endif // SHOW_PROCESSING_TIME_

#ifdef SHOW_PROCESSING_TIME_
			cout << "DetectLEDMarker() ROI ON" << endl;
			cout << "DetectLEDMarker() ROI ON Time	:" << detecttimea << endl;
#endif // SHOW_PROCESSING_TIME_
		}

		//4つのLEDから位置姿勢計算
		///理想ピクセル座標系に変換
#ifdef SHOW_PROCESSING_TIME_
		QueryPerformanceCounter(&detectstart);
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
		//計算された位置に連続性が確認されないときはエラーとする
#ifdef SHOW_PROCESSING_TIME_
		QueryPerformanceCounter(&detectend);
		detecttimee = (double)(detectend.QuadPart - detectstart.QuadPart) / freq.QuadPart;
		cout << "DetectLEDMarker() CalcPose		:" << detecttimee << endl;
#endif // SHOW_PROCESSING_TIME_

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