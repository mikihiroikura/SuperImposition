#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <cstring>
#include <iostream>

#pragma warning(disable:4996)
using namespace std;

int main() {
	string img0_dir = "frame00329.png";
	string img1_dir = "frame00330.png";
	cv::Mat img0 = cv::imread(img0_dir);
	cv::Mat img1 = cv::imread(img1_dir);
	cv::Mat diffimg;

	cv::absdiff(img0, img1, diffimg);


	return 0;
}