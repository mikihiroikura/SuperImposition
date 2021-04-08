#define _USE_MATH_DEFINES

#include <HSC/KAYACoaXpressClass.hpp>
#include <vector>
#include <time.h>
#include <thread>
#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/highgui.hpp>
#include "realsense.h"
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtc/matrix_transform.hpp> 
#include <glm/gtx/transform.hpp>
#include <cmath>

#ifdef _DEBUG
#define LIB_EXT "d.lib"
#else
#define LIB_EXT ".lib"
#endif

//�J�����Ɋւ���p�����[�^
int width = 896;
int height = 896;
float fps = 1000.0;
float exposuretime = 912.0;
int offsetx = 512;
int offsety = 92;
const int multicnt = 2;
double map_coeff[4], stretch_mat[4], det, distort[4];
cv::Mat in_img_hsc, in_img_hsc_multi;
vector<cv::Mat> in_imgs_hsc_onoff;
vector<cv::Mat> save_img_hsc;
uint8_t* in_img_hsc_multi_src;


//RealSense�Ɋւ���p�����[�^
cv::Mat in_img_rs;
vector<cv::Mat> save_img_rs;
rs2::context context;
const unsigned int colorwidth = 848;
const unsigned int colorheight = 480;
const unsigned int colorfps = 60;
const unsigned int depthwidth = 848;
const unsigned int depthheight = 480;
const unsigned int depthfps = 60;

//�}�[�J���o�Ɋւ���p�����[�^
int detectid = 0;
vector<cv::Mat> detectimg;
cv::Mat diffimg, diffimg_hsv, detectimg_on_hsv;
const cv::Scalar greenLED_min(0, 150, 0);
const cv::Scalar greenLED_max(256, 256, 256);
const cv::Scalar blueLED_min(150, 0, 0);
const cv::Scalar blueLED_max(256, 256, 256);
const cv::Scalar HSVLED_min(0, 0, 100);
const cv::Scalar HSVLED_max(256, 256, 256);
double ledmass[4] = { 0 }, ledmomx[4] = { 0 }, ledmomy[4] = { 0 };
const int roi_led_minx_ini[4] = { width, width, width, width },
roi_led_maxx_ini[4] = { 0 }, roi_led_miny_ini[4] = { height, height, height, height }, roi_led_maxy_ini[4] = { 0 };
int roi_led_minx[4], roi_led_maxx[4], roi_led_miny[4], roi_led_maxy[4];
bool leddetected = false;
int ptscnt;
float* ptsptr, * ptscand_ptr, * center_src;
cv::Mat pts, ptscand, labels, centers, afterlabel;
double dist, dist_cluster_thr = 20, dist_centers_thr = 10;
uint8_t* diffimg_src, * detectimg0_src, * detectimg1_src, * detectimg_on_src, * detectimghsv_on_src, * afterlabel_src;
int on_img_id, on_img_cnt, blueno = -1, labelno;
int32_t* labelptr, * labelptr_debug;
int greenbluecnt[4][2] = { 0 };
int32_t h_on;
vector<cv::Rect> rois, rois_rand;
const int roi_led_margin = 10;
double ledimpos[4][2] = { 0 }, ledidimpos[4][2] = { 0 }, ledimpos_rand[4][2] = { 0 };
double ledcog[2] = { 0 };
double thetamax, thetamin, thetamaxid, thetaminid;
double cross, dot, theta[3];
double phi, w, lambda;
double ledcamdir[4][3] = { 0 };
double lednormdir[4][3] = { 0 };
glm::mat4 RTm2c = glm::mat4(1.0);
glm::mat4* RTm2c_buffer, * RTm2c_toGPU;
int RTm2c_bufferid = 0, RTm2c_outid = 0;
cv::Mat A = cv::Mat::zeros(12, 7, CV_64F);
cv::Mat x = cv::Mat::zeros(7, 1, CV_64F);
cv::Mat b = cv::Mat::zeros(12, 1, CV_64F);
double* Asrc = A.ptr<double>(0);
double* xsrc = x.ptr<double>(0);
double* bsrc = b.ptr<double>(0);
const double markeredge = 235 / 2 * 0.001;//�P�ʂ�m
const double markerpos[4][2] = { {markeredge, markeredge}, {-markeredge, markeredge}, {-markeredge, -markeredge}, {markeredge, -markeredge} };
int detectresult = -1;

#define VIDEO_MODE_
//#define IMG_MODE_
#define GET_HSC
#define GET_RS

#pragma comment(lib,"KAYACoaXpressLib" LIB_EXT)
#pragma warning(disable:4996)
using namespace std;

void TakePicture(kayacoaxpress* cam, bool* flg);
void GetImgsRS(realsense* rs, bool* flg);
int DetectLEDMarker();



int main() {

#ifdef GET_HSC
	//�J�����̃C���X�^���X����
	kayacoaxpress cam;
	cam.connect(1);
	//�p�����[�^�̐ݒ�
	cam.setParam(paramTypeKAYACoaXpress::AcquisitionMode::TriggerMode, 1); //�g���K�[���[�h�ŋN��
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
	in_img_hsc_multi = cv::Mat(cam.getParam(paramTypeCamera::paramInt::HEIGHT) * multicnt, cam.getParam(paramTypeCamera::paramInt::WIDTH), CV_8UC3, cv::Scalar::all(255));
	in_img_hsc_multi_src = in_img_hsc_multi.ptr<uint8_t>(0);
	for (size_t i = 0; i < multicnt; i++)
	{
		detectimg.push_back(in_img_hsc.clone());
	}
	//�J���������p�����[�^Calibration���ʂ̌Ăяo��
	FILE* fcam;
	fcam = fopen("202101070034_fisheyeparam_cam0.csv", "r");
	for (size_t i = 0; i < 4; i++) { fscanf(fcam, "%lf,", &map_coeff[i]); }
	for (size_t i = 0; i < 4; i++) { fscanf(fcam, "%lf,", &stretch_mat[i]); }
	swap(stretch_mat[1], stretch_mat[2]);
	for (size_t i = 0; i < 2; i++) { fscanf(fcam, "%lf,", &distort[i]); }
	fclose(fcam);
	det = 1 / (stretch_mat[0] - stretch_mat[1] * stretch_mat[2]);
#endif // GET_HSC

#ifdef GET_RS
	//RealSense�̏�����
	cout << "Set RealsenseD435..........";
	const rs2::device_list device_list = context.query_devices();
	rs2::device device = device_list[0];
	realsense rs_device(device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER), RS2_FORMAT_BGR8,
		colorwidth, colorheight, colorfps, RS2_FORMAT_Z16, depthwidth, depthheight, depthfps);
	cout << "OK!" << endl;
	in_img_rs = cv::Mat(colorheight, colorwidth, CV_8UC3, cv::Scalar::all(255));
#endif // GET_RS

	//�ʒu�p���v�Z�p�̕ϐ��ݒ�
	//�P�_�ۑ��p�s��̍쐬
	ptscand = cv::Mat::zeros(width * height, 1, CV_32FC2);
	ptscand_ptr = ptscand.ptr<float>(0);
	//ROI�̐ݒ�
	for (size_t i = 0; i < 4; i++)
	{
		rois.push_back(cv::Rect(0, 0, width, height));
		rois_rand.push_back(cv::Rect(0, 0, width, height));
	}
	cv::Mat zero = cv::Mat(cam.getParam(paramTypeCamera::paramInt::HEIGHT), cam.getParam(paramTypeCamera::paramInt::WIDTH), CV_8UC3, cv::Scalar::all(0));
	diffimg = zero.clone();
	diffimg_src = diffimg.ptr<uint8_t>(0);

	//����ۑ��p�̃t�@�C���쐬
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
	sprintf(buff, "%04d%02d%02d%02d%02d_video_rs.mp4", 1900 + pnow->tm_year, 1 + pnow->tm_mon, pnow->tm_mday, pnow->tm_hour, pnow->tm_min);
	cv::VideoWriter video_rs(save_dir + buff, cv::VideoWriter::fourcc('M', 'P', '4', 'V'), 30, cv::Size(colorwidth, colorheight), true);
	if (!video_rs.isOpened()) {
		cout << "Video cannot be opened..." << endl;
		return 1;
	}
#endif // GET_RS
#endif // VIDEO_MODE_


	//Thread�Z�b�g
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
		//���݂̉摜��\��
#ifdef GET_HSC
		cv::imshow("img cam", detectimg[0]);
#endif // GET_HSC
#ifdef GET_RS
		cv::imshow("img realsense", in_img_rs);
#endif // GET_RS

		//�ʒu�p���v�Z
		detectresult = DetectLEDMarker();

		//30fps�ɂȂ�悤�Ɏ��Ԍv��
		int key = cv::waitKey(33);
		if (key == 'q') break;

		//VideoCapture
#ifdef VIDEO_MODE_
		if (key == 's') videocapflg = true;
		if (key == 'f') videocapflg = false;
#ifdef GET_HSC
		if (videocapflg) video_hsc.write(detectimg[0].clone());
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
		cam->captureFrame(in_img_hsc_multi_src, multicnt);
		memcpy(detectimg[0].data, in_img_hsc_multi_src, height * width * 3);
		memcpy(detectimg[1].data, in_img_hsc_multi_src + height * width * 3, height * width * 3);
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

int DetectLEDMarker() {
	//LED�������o�̎��́C�摜�S�̂�T������
	if (detectimg[0].data != NULL && detectimg[1].data != NULL && (int)detectimg[0].data[0] != 255 && (int)detectimg[1].data[0] != 255 && (int)detectimg[0].data[0] != 0 && (int)detectimg[1].data[0] != 0)
	{
		//�N���X�^�[���ƂɋP�x�d�S���v�Z����
		for (size_t i = 0; i < 4; i++)
		{
			ledmass[i] = 0, ledmomx[i] = 0, ledmomy[i] = 0;
		}
		memcpy(roi_led_minx, roi_led_minx_ini, sizeof(roi_led_minx_ini));
		memcpy(roi_led_maxx, roi_led_maxx_ini, sizeof(roi_led_maxx_ini));
		memcpy(roi_led_miny, roi_led_miny_ini, sizeof(roi_led_miny_ini));
		memcpy(roi_led_maxy, roi_led_maxy_ini, sizeof(roi_led_maxy_ini));

		//4�S�Ă�LED�����o���Ă��Ȃ���
		if (!(leddetected))
		{
			//�����摜�̐���
			cv::absdiff(detectimg[0], detectimg[1], diffimg);
			//HSV��V��臒l����
			diffimg_src = diffimg.ptr<uint8_t>(0);
			ptscnt = 0;
			for (size_t i = 0; i < width; i++)
			{
				for (size_t j = 0; j < height; j++)
				{
					if ((uint8_t)diffimg_src[j * width * 3 + i * 3] > HSVLED_min(2) || (uint8_t)diffimg_src[j * width * 3 + i * 3 + 1] > HSVLED_min(2) || (uint8_t)diffimg_src[j * width * 3 + i * 3 + 2] > HSVLED_min(2))
					{
						ptscand_ptr[ptscnt * 2 + 0] = (float)i;
						ptscand_ptr[ptscnt * 2 + 1] = (float)j;
						ptscnt++;
					}
				}
			}

			//�����ō����摜����P�_��������Ȃ��Ƃ��̗�O����������
			if (ptscnt <= 0) return 6;

			//�P�x�̍����_�Q��4�����ɃN���X�^�����O
			pts = ptscand(cv::Rect(0, 0, 1, ptscnt));
			cv::kmeans(pts, 4, labels, cvTermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1.0), 1, cv::KMEANS_PP_CENTERS, centers);
			center_src = centers.ptr<float>(0);
			//�N���X�^�Ԃ̋�����臒l�ȉ��Ȃ�Ζ����o�Ɣ���
			for (size_t i = 0; i < 4; i++)
			{
				for (size_t j = 0; j < 4; j++)
				{
					if (i != j)
					{
						dist = hypot(center_src[i * 2 + 0] - center_src[j * 2 + 0], center_src[i * 2 + 1] - center_src[j * 2 + 1]);
						if (dist < dist_centers_thr) {
							return 1;
						}
					}
				}
			}

			//ON�摜���ǂ��炩���肷��
			detectimg0_src = detectimg[0].ptr<uint8_t>(0);
			detectimg1_src = detectimg[1].ptr<uint8_t>(0);
			on_img_cnt = 0;
			for (size_t i = 0; i < ptscnt; i++)
			{
				if ((int32_t)detectimg0_src[(int)ptscand_ptr[i * 2 + 1] * width * 3 + (int)ptscand_ptr[i * 2 + 0] * 3] > (int32_t)detectimg1_src[(int)ptscand_ptr[i * 2 + 1] * width * 3 + (int)ptscand_ptr[i * 2 + 0] * 3])
				{//2���̉摜�ŋP�x�l���r
					on_img_cnt++;
				}
			}
			if (on_img_cnt > ptscnt / 2) on_img_id = 0;
			else on_img_id = 1;
			detectimg_on_src = detectimg[on_img_id].ptr<uint8_t>(0);

			//���ނ��Ƃɐ΂̌��̃J�E���g
			blueno = -1;
			cv::cvtColor(detectimg[on_img_id], detectimg_on_hsv, CV_BGR2HSV);
			detectimghsv_on_src = detectimg_on_hsv.ptr<uint8_t>(0);
			labelptr = labels.ptr<int32_t>(0);
			memset(greenbluecnt, 0, sizeof(int) * 4 * 2);
			for (size_t i = 0; i < ptscnt; i++)
			{
				h_on = (int32_t)detectimghsv_on_src[(int)ptscand_ptr[i * 2 + 1] * width * 3 + (int)ptscand_ptr[i * 2 + 0] * 3];
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
			if (blueno == -1) {
				return 5;
			}

			//�N���X�^���ƂɋP�x�d�S�̌v�Z
			for (size_t i = 0; i < ptscnt; i++)
			{
				labelno = (int)labelptr[i];
				dist = hypot(center_src[labelno * 2 + 0] - (int)ptscand_ptr[i * 2 + 0], center_src[labelno * 2 + 1] - (int)ptscand_ptr[i * 2 + 1]);
				if (dist < dist_cluster_thr)
				{
					if (labelno == blueno)
					{
						if ((int32_t)detectimg_on_src[(int)ptscand_ptr[i * 2 + 1] * width * 3 + (int)ptscand_ptr[i * 2 + 0] * 3] > blueLED_min(0))
						{//On�摜�̐�臒l�͂����ƍ���
							ledmass[labelno] += (double)detectimg_on_src[(int)ptscand_ptr[i * 2 + 1] * width * 3 + (int)ptscand_ptr[i * 2 + 0] * 3];
							ledmomx[labelno] += (double)detectimg_on_src[(int)ptscand_ptr[i * 2 + 1] * width * 3 + (int)ptscand_ptr[i * 2 + 0] * 3] * (int)ptscand_ptr[i * 2 + 0];
							ledmomy[labelno] += (double)detectimg_on_src[(int)ptscand_ptr[i * 2 + 1] * width * 3 + (int)ptscand_ptr[i * 2 + 0] * 3] * (int)ptscand_ptr[i * 2 + 1];
							//ROI���v�Z
							if (roi_led_maxx[labelno] < (int)ptscand_ptr[i * 2 + 0]) roi_led_maxx[labelno] = (int)ptscand_ptr[i * 2 + 0];
							if (roi_led_minx[labelno] > (int)ptscand_ptr[i * 2 + 0]) roi_led_minx[labelno] = (int)ptscand_ptr[i * 2 + 0];
							if (roi_led_maxy[labelno] < (int)ptscand_ptr[i * 2 + 1]) roi_led_maxy[labelno] = (int)ptscand_ptr[i * 2 + 1];
							if (roi_led_miny[labelno] > (int)ptscand_ptr[i * 2 + 1]) roi_led_miny[labelno] = (int)ptscand_ptr[i * 2 + 1];
						}
					}
					else
					{//On�摜�̗΂�臒l�͂����ƍ���
						if ((int32_t)detectimg_on_src[(int)ptscand_ptr[i * 2 + 1] * width * 3 + (int)ptscand_ptr[i * 2 + 0] * 3 + 1] > greenLED_min(1))
						{
							ledmass[labelno] += (double)detectimg_on_src[(int)ptscand_ptr[i * 2 + 1] * width * 3 + (int)ptscand_ptr[i * 2 + 0] * 3 + 1];
							ledmomx[labelno] += (double)detectimg_on_src[(int)ptscand_ptr[i * 2 + 1] * width * 3 + (int)ptscand_ptr[i * 2 + 0] * 3 + 1] * (int)ptscand_ptr[i * 2 + 0];
							ledmomy[labelno] += (double)detectimg_on_src[(int)ptscand_ptr[i * 2 + 1] * width * 3 + (int)ptscand_ptr[i * 2 + 0] * 3 + 1] * (int)ptscand_ptr[i * 2 + 1];
							//ROI���v�Z
							if (roi_led_maxx[labelno] < (int)ptscand_ptr[i * 2 + 0]) roi_led_maxx[labelno] = (int)ptscand_ptr[i * 2 + 0];
							if (roi_led_minx[labelno] > (int)ptscand_ptr[i * 2 + 0]) roi_led_minx[labelno] = (int)ptscand_ptr[i * 2 + 0];
							if (roi_led_maxy[labelno] < (int)ptscand_ptr[i * 2 + 1]) roi_led_maxy[labelno] = (int)ptscand_ptr[i * 2 + 1];
							if (roi_led_miny[labelno] > (int)ptscand_ptr[i * 2 + 1]) roi_led_miny[labelno] = (int)ptscand_ptr[i * 2 + 1];
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
			}

			//���ԃo���o���ł�LED�̋P�x�d�S�v�Z
			for (size_t i = 0; i < 4; i++)
			{
				//�N���X�^������臒l�ȏ�̋P�_�����݂��Ȃ��Ƃ��͖����o�ŏI��
				if (ledmass[i] <= 0) {
					return 2;
				}
				ledimpos_rand[i][0] = ledmomx[i] / ledmass[i];
				ledimpos_rand[i][1] = ledmomy[i] / ledmass[i];
			}

			//�F���玞�v���ɗ�LED�𓖂Ă͂߂�
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
		}

		//�ƗΗ������o���Ă���Ƃ�
		else
		{
			//ON�摜���ǂ��炩���肷��
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
						{//2���̉摜�ŋP�x�l���r
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
								//ROI���v�Z
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
								//ROI���v�Z
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
					//ROI������臒l�ȏ�̋P�_�����݂��Ȃ��Ƃ��ɏI��
					leddetected = false;
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
		}

		//4��LED����ʒu�p���v�Z
		///���z�s�N�Z�����W�n�ɕϊ�
		for (size_t i = 0; i < 4; i++)
		{
			ledidimpos[i][0] = det * ((ledimpos[i][0] - distort[0]) - stretch_mat[1] * (ledimpos[i][1] - distort[1]));
			ledidimpos[i][1] = det * (-stretch_mat[2] * (ledimpos[i][0] - distort[0]) + stretch_mat[0] * (ledimpos[i][1] - distort[1]));
		}
		///���z�s�N�Z��->�����x�N�g��
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

		//4�̕����x�N�g������C�Ζʂ̖@���x�N�g�������߂�
		for (size_t i = 0; i < 4; i++)
		{
			lednormdir[i][0] = ledcamdir[i][1] * ledcamdir[(i + 1) % 4][2] - ledcamdir[i][2] * ledcamdir[(i + 1) % 4][1];
			lednormdir[i][1] = ledcamdir[i][2] * ledcamdir[(i + 1) % 4][0] - ledcamdir[i][0] * ledcamdir[(i + 1) % 4][2];
			lednormdir[i][2] = ledcamdir[i][0] * ledcamdir[(i + 1) % 4][1] - ledcamdir[i][1] * ledcamdir[(i + 1) % 4][0];
		}

		//�@���x�N�g������CLED�}�[�J�̕ӂ̕����x�N�g����2���߂�
		for (size_t i = 0; i < 2; i++)
		{
			RTm2c[0][i] = -(lednormdir[i][1] * lednormdir[(i + 2)][2] - lednormdir[i][2] * lednormdir[(i + 2)][1]);
			RTm2c[1][i] = -(lednormdir[i][2] * lednormdir[(i + 2)][0] - lednormdir[i][0] * lednormdir[(i + 2)][2]);
			RTm2c[2][i] = -(lednormdir[i][0] * lednormdir[(i + 2)][1] - lednormdir[i][1] * lednormdir[(i + 2)][0]);
			lambda = 1 / pow(pow(RTm2c[0][i], 2) + pow(RTm2c[1][i], 2) + pow(RTm2c[2][i], 2), 0.5);
			RTm2c[0][i] *= lambda;
			RTm2c[1][i] *= lambda;
			RTm2c[2][i] *= lambda;
		}

		//�J����-�}�[�J�Ԃ̑��Ύp���̌v�Z(�c��̕����x�N�g�����O�ςŋ��߂�)
		RTm2c[0][2] = RTm2c[1][0] * RTm2c[2][1] - RTm2c[2][0] * RTm2c[1][1];
		RTm2c[1][2] = RTm2c[2][0] * RTm2c[0][1] - RTm2c[0][0] * RTm2c[2][1];
		RTm2c[2][2] = RTm2c[0][0] * RTm2c[1][1] - RTm2c[1][0] * RTm2c[0][1];
		lambda = 1 / pow(pow(RTm2c[0][2], 2) + pow(RTm2c[1][2], 2) + pow(RTm2c[2][2], 2), 0.5);
		RTm2c[0][2] *= lambda;
		RTm2c[1][2] *= lambda;
		RTm2c[2][2] *= lambda;

		//�����ŁC�����x�N�g�����摜�����̌덷���悹�Ē������Ȃ��Ƃ��ɋ����ɒ�����������x�N�g�����v�Z����
		RTm2c[0][1] = RTm2c[1][2] * RTm2c[2][0] - RTm2c[2][2] * RTm2c[1][0];
		RTm2c[1][1] = RTm2c[2][2] * RTm2c[0][0] - RTm2c[0][2] * RTm2c[2][0];
		RTm2c[2][1] = RTm2c[0][2] * RTm2c[1][0] - RTm2c[1][2] * RTm2c[0][0];
		lambda = 1 / pow(pow(RTm2c[0][1], 2) + pow(RTm2c[1][1], 2) + pow(RTm2c[2][1], 2), 0.5);
		RTm2c[0][1] *= lambda;
		RTm2c[1][1] *= lambda;
		RTm2c[2][1] *= lambda;

		//���Ⴢ�f���Ƒ��Ύp����p���ăJ����-�}�[�J�Ԃ̑��Έʒu���v�Z
		for (size_t i = 0; i < 4; i++)
		{
			Asrc[i * 7 * 3 + i] = ledcamdir[i][0];
			Asrc[i * 7 * 3 + 7 + i] = ledcamdir[i][1];
			Asrc[i * 7 * 3 + 14 + i] = ledcamdir[i][2];
			Asrc[i * 7 * 3 + 4] = -1;
			Asrc[i * 7 * 3 + 12] = -1;
			Asrc[i * 7 * 3 + 20] = -1;
			bsrc[i * 3 + 0] = RTm2c[0][0] * markerpos[i][0] + RTm2c[0][1] * markerpos[i][1];
			bsrc[i * 3 + 1] = RTm2c[1][0] * markerpos[i][0] + RTm2c[1][1] * markerpos[i][1];
			bsrc[i * 3 + 2] = RTm2c[2][0] * markerpos[i][0] + RTm2c[2][1] * markerpos[i][1];
		}
		x = A.inv(cv::DECOMP_SVD) * b;
		RTm2c[0][3] = xsrc[4];
		RTm2c[1][3] = xsrc[5];
		RTm2c[2][3] = xsrc[6];
		//�v�Z���ꂽ�ʒu�ɘA�������m�F����Ȃ��Ƃ��̓G���[�Ƃ���

		return 0;
		//�ʒu�p�����v�Z�ł��Đ���I��
	}
	else
	{
		//���͉摜���ُ�ł���Ƃ��ɁC�����I��
		leddetected = false;
		return 4;
	}
}