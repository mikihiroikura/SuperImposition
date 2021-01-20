#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <cstring>
#include <iostream>

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
uint8_t* diffimg_src, * detectimg_on_src;
cv::Mat labels;

/// カメラパラメータ
const int width = 896;
const int height = 896;
const float fps = 1000.0;
const float exposuretime = 912.0;
const int offsetx = 512;
const int offsety = 92;

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


	//輝度の高い点群を4か所にクラスタリング
	cv::Mat pts = cv::Mat::zeros(Vpts.size(), 1, CV_32FC2);
	float* ptsptr = pts.ptr<float>(0);
	for (size_t i = 0; i < Vpts.size(); i++)
	{
		ptsptr[i * 2 + 0] = (float)Vpts[i].x;
		ptsptr[i * 2 + 1] = (float)Vpts[i].y;
	}
	cv::kmeans(pts, 4, labels, cvTermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1.0), 1, cv::KMEANS_PP_CENTERS);
	cv::Mat afterlabel = cv::Mat(896, 896, CV_8UC3, cv::Scalar::all(0));
	uint8_t* afterlabel_src = afterlabel.ptr<uint8_t>(0);
	int32_t* labelptr = labels.ptr<int32_t>(0);
	for (size_t i = 0; i < Vpts.size(); i++)
	{
		//cout << (int32_t)labelptr[i] << endl;
		if ((int32_t)labelptr[i] == 0)
		{
			afterlabel_src[Vpts[i].y * width * 3 + Vpts[i].x * 3] = 0;
		}
	}

	return 0;
}