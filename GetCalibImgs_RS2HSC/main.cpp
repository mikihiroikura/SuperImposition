#include <HSC/KAYACoaXpressClass.hpp>
#include <vector>
#include <time.h>
#include <thread>
#include "realsense.h"

#ifdef _DEBUG
#define LIB_EXT "d.lib"
#else
#define LIB_EXT ".lib"
#endif

//カメラに関するパラメータ
int width = 896;
int height = 896;
float fps = 1000.0;
float exposuretime = 912.0;
int offsetx = 512;
int offsety = 92;


//RealSenseに関するパラメータ
rs2::context context;
int rsid = 1;
const unsigned int colorwidth = 848;
const unsigned int colorheight = 480;
const unsigned int colorfps = 60;
const unsigned int depthwidth = 848;
const unsigned int depthheight = 480;
const unsigned int depthfps = 60;

#define VIDEO_MODE_
//#define IMG_MODE_
//#define GET_HSC
#define GET_RS

#pragma comment(lib,"KAYACoaXpressLib" LIB_EXT)
#pragma warning(disable:4996)
using namespace std;

void TakePicture(kayacoaxpress* cam, bool* flg);
void GetImgsRS(realsense* rs, bool* flg);

cv::Mat in_img_hsc, in_img_rs;
vector<cv::Mat> save_img_hsc, save_img_rs;

int main() {
	
#ifdef GET_HSC
	//カメラのインスタンス生成
	kayacoaxpress cam;
	cam.connect(1);
	//パラメータの設定
	cam.setParam(paramTypeCamera::paramInt::WIDTH, width);
	cam.setParam(paramTypeCamera::paramInt::HEIGHT, height);
	cam.setParam(paramTypeKAYACoaXpress::paramInt::OffsetX, offsetx);
	cam.setParam(paramTypeKAYACoaXpress::paramInt::OffsetY, offsety);
	cam.setParam(paramTypeCamera::paramFloat::FPS, fps);
	cam.setParam(paramTypeKAYACoaXpress::paramFloat::ExposureTime, exposuretime);
	cam.setParam(paramTypeKAYACoaXpress::CaptureType::BayerGRGrab);
	cam.setParam(paramTypeKAYACoaXpress::Gain::x2);
	cam.parameter_all_print();
	in_img_hsc = cv::Mat(cam.getParam(paramTypeCamera::paramInt::HEIGHT), cam.getParam(paramTypeCamera::paramInt::WIDTH), CV_8UC3, cv::Scalar::all(255));
#endif // GET_HSC

#ifdef GET_RS
	//RealSenseの初期化
	cout << "Set RealsenseD435..........";
	const rs2::device_list device_list = context.query_devices();
	rs2::device device = device_list[rsid];
	realsense rs_device(device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER), RS2_FORMAT_BGR8,
		colorwidth, colorheight, colorfps, RS2_FORMAT_Z16, depthwidth, depthheight, depthfps);
	cout << "OK!" << endl;
	in_img_rs = cv::Mat(colorheight, colorwidth, CV_8UC3, cv::Scalar::all(255));
#endif // GET_RS

	//動画保存用のファイル作成
	string save_dir = "D:\\Github_output\\SuperImposition\\GetCalibImgs_RS2HSC\\";
	time_t now = time(NULL);
	struct tm* pnow = localtime(&now);
	char buff[128];
#ifdef VIDEO_MODE_
#ifdef GET_HSC
	sprintf(buff, "%04d%02d%02d%02d%02d_video_hsc.mp4", 1900 + pnow->tm_year, 1 + pnow->tm_mon, pnow->tm_mday, pnow->tm_hour, pnow->tm_min);
	cv::VideoWriter video_hsc(save_dir + buff, cv::VideoWriter::fourcc('M', 'P', '4', 'V'), 30, cv::Size(cam.getParam(paramTypeCamera::paramInt::WIDTH), cam.getParam(paramTypeCamera::paramInt::HEIGHT)), true);
	if (!video_hsc.isOpened()) {
		cout << "Video cannot be opened..." << endl;
		return 1;
	}
#endif // GET_HSC
#ifdef GET_RS
	sprintf(buff, "%04d%02d%02d%02d%02d_video_rs%01d.mp4", 1900 + pnow->tm_year, 1 + pnow->tm_mon, pnow->tm_mday, pnow->tm_hour, pnow->tm_min,rsid);
	cv::VideoWriter video_rs(save_dir + buff, cv::VideoWriter::fourcc('M', 'P', '4', 'V'), 30, cv::Size(colorwidth, colorheight), true);
	if (!video_rs.isOpened()) {
		cout << "Video cannot be opened..." << endl;
		return 1;
	}
#endif // GET_RS
#endif // VIDEO_MODE_
	

	//Threadセット
	bool flg = true;
#ifdef GET_HSC
	cam.start();
	thread thr_cam(TakePicture, &cam, &flg);
#endif // GET_HSC
#ifdef GET_RS
	thread thr_rs(GetImgsRS, &rs_device, &flg);
#endif // GET_RS
	bool videocapflg = false;

	while (true)
	{
		//現在の画像を表示
#ifdef GET_HSC
		cv::imshow("img cam", in_img_hsc);
#endif // GET_HSC
#ifdef GET_RS
		cv::imshow("img realsense", in_img_rs);
#endif // GET_RS

		//30fpsになるように時間計測
		int key = cv::waitKey(33);
		if (key == 'q') break;

		//VideoCapture
#ifdef VIDEO_MODE_
		if (key == 's') videocapflg = true;
		if (key == 'f') videocapflg = false;
#ifdef GET_HSC
		if (videocapflg) video_hsc.write(in_img_hsc.clone());
#endif // GET_HSC
#ifdef GET_RS
		if (videocapflg) video_rs.write(in_img_rs.clone());
#endif // GET_RS

		
#endif // VIDEO_MODE_
#ifdef IMG_MODE_
#ifdef GET_HSC
		if (key == 's') save_img_hsc.push_back(in_img_hsc.clone());
#endif // GET_HSC
#ifdef GET_RS
		if (key == 's') save_img_rs.push_back(in_img_rs.clone());
#endif // GET_RS
#endif // IMG_MODE_


	}
	flg = false;
#ifdef GET_HSC
	if (thr_cam.joinable()) thr_cam.join();
#ifdef VIDEO_MODE_
	video_hsc.release();
#endif // VIDEO_MODE_
#ifdef IMG_MODE_
	for (size_t i = 0; i < save_img_hsc.size(); i++)
	{
		sprintf(buff, "%04d%02d%02d%02d%02d_img_hsc%02d.png", 1900 + pnow->tm_year, 1 + pnow->tm_mon, pnow->tm_mday, pnow->tm_hour, pnow->tm_min, (int)i);
		cv::imwrite(save_dir + buff, save_img_hsc[i]);
	}
#endif // IMG_MODE_
	cam.stop();
	cam.disconnect();
#endif // GET_HSC


#ifdef GET_RS
	if (thr_rs.joinable()) thr_rs.join();
#ifdef VIDEO_MODE_
	video_rs.release();
#endif // VIDEO_MODE_
#ifdef IMG_MODE_
	for (size_t i = 0; i < save_img_rs.size(); i++)
	{
		sprintf(buff, "%04d%02d%02d%02d%02d_img_rs%02d.png", 1900 + pnow->tm_year, 1 + pnow->tm_mon, pnow->tm_mday, pnow->tm_hour, pnow->tm_min, (int)i);
		cv::imwrite(save_dir + buff, save_img_rs[i]);
	}
#endif // IMG_MODE_
#endif // GET_RS




	return 0;
}

void TakePicture(kayacoaxpress* cam, bool* flg) {
	while (*flg)
	{
		cam->captureFrame(in_img_hsc.data);
	}
}

void GetImgsRS(realsense* rs, bool* flg) {
	while (*flg)
	{
		rs->update_frame();
		rs->update_color();
		rs->transform_color_img();
		memcpy(in_img_rs.data, rs->colorimg.data, colorwidth * colorheight * 3);
	}
}