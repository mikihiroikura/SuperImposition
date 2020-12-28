#include <HSC/KAYACoaXpressClass.hpp>
#include <vector>
#include <time.h>
#include <thread>

#ifdef _DEBUG
#define LIB_EXT "d.lib"
#else
#define LIB_EXT ".lib"
#endif


#pragma comment(lib,"KAYACoaXpressLib" LIB_EXT)
#pragma warning(disable:4996)
using namespace std;

void TakePicture(kayacoaxpress* cam, bool* flg);

cv::Mat in_img;

int main() {
	//カメラパラメータ
	int width = 896;
	int height = 896;
	float fps = 1000.0;
	float exposuretime = 912.0;
	int offsetx = 480;
	int offsety = 92;

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

	//画像出力用Mat
	in_img = cv::Mat(cam.getParam(paramTypeCamera::paramInt::HEIGHT), cam.getParam(paramTypeCamera::paramInt::WIDTH), CV_8UC3, cv::Scalar::all(255));

	//動画保存用のファイル作成
	string save_dir = "D:\\Github_output\\SuperImpositionHSC\\GetCalibImgs\\";
	time_t now = time(NULL);
	struct tm* pnow = localtime(&now);
	char buff[128];
	sprintf(buff, "%04d%02d%02d%02d%02d_video.mp4", 1900 + pnow->tm_year, 1 + pnow->tm_mon, pnow->tm_mday, pnow->tm_hour, pnow->tm_min);
	save_dir += buff;
	cv::VideoWriter video(save_dir, cv::VideoWriter::fourcc('M', 'P', '4', 'V'), 30, cv::Size(cam.getParam(paramTypeCamera::paramInt::WIDTH), cam.getParam(paramTypeCamera::paramInt::HEIGHT)), true);
	if (!video.isOpened()) {
		cout << "Video cannot be opened..." << endl;
		return 1;
	}

	//カメラ起動
	cam.start();

	//Threadセット
	bool flg = true;
	thread thr(TakePicture, &cam, &flg);
	bool videocapflg = false;


	while (true)
	{
		//現在の画像を表示
		cv::imshow("img", in_img);

		//30fpsになるように時間計測
		int key = cv::waitKey(33);
		if (key == 'q') break;

		//VideoCapture
		if (key == 's') videocapflg = true;
		if (key == 'f') videocapflg = false;
		if (videocapflg) video.write(in_img.clone());
	}
	flg = false;
	if (thr.joinable()) thr.join();

	video.release();

	cam.stop();
	cam.disconnect();

	return 0;
}

void TakePicture(kayacoaxpress* cam, bool* flg) {
	while (*flg)
	{
		cam->captureFrame(in_img.data);
	}
}