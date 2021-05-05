#define _CRT_SECURE_NO_WARNINGS

#include "realsense.h"

//RealSenseに関するパラメータ
rs2::context context;
int ugvrsid = 0, uavrsid = 1;
const unsigned int colorwidth = 848;
const unsigned int colorheight = 480;
const unsigned int colorfps = 60;
const unsigned int depthwidth = 848;
const unsigned int depthheight = 480;
const unsigned int depthfps = 60;

int main() {
	//RealSenseの初期化
	cout << "Set RealsenseD435..........";
	const rs2::device_list device_list = context.query_devices();
	rs2::device ugvdevice = device_list[ugvrsid];
	realsense ugvrs_device(ugvdevice.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER), RS2_FORMAT_BGR8,
		colorwidth, colorheight, colorfps, RS2_FORMAT_Z16, depthwidth, depthheight, depthfps);
	cout << "OK!" << endl;


	cout << "Set RealsenseD435..........";
	rs2::device uavdevice = device_list[uavrsid];
	realsense uavrs_device(uavdevice.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER), RS2_FORMAT_BGR8,
		colorwidth, colorheight, colorfps, RS2_FORMAT_Z16, depthwidth, depthheight, depthfps);
	cout << "OK!" << endl;


	//ここにRSの内部パラを呼び出す
	rs2::stream_profile ugvrs_stream_color = ugvrs_device.pipe.get_active_profile().get_stream(RS2_STREAM_COLOR);
	rs2::stream_profile uavrs_stream_color = uavrs_device.pipe.get_active_profile().get_stream(RS2_STREAM_COLOR);
	rs2::stream_profile ugvrs_stream_depth = ugvrs_device.pipe.get_active_profile().get_stream(RS2_STREAM_DEPTH);
	rs2::stream_profile uavrs_stream_depth = uavrs_device.pipe.get_active_profile().get_stream(RS2_STREAM_DEPTH);
	rs2_intrinsics ugvrs_intparams = ugvrs_stream_color.as<rs2::video_stream_profile>().get_intrinsics();
	rs2_intrinsics uavrs_intparams = uavrs_stream_color.as<rs2::video_stream_profile>().get_intrinsics();
	rs2_intrinsics ugvrs_intparams_depth = ugvrs_stream_depth.as<rs2::video_stream_profile>().get_intrinsics();
	rs2_intrinsics uavrs_intparams_depth = uavrs_stream_depth.as<rs2::video_stream_profile>().get_intrinsics();
	rs2_extrinsics ugvrs_extparams_col2dpt = ugvrs_stream_color.get_extrinsics_to(ugvrs_stream_depth);
	rs2_extrinsics uavrs_extparams_col2dpt = uavrs_stream_color.get_extrinsics_to(uavrs_stream_depth);


	//内部パラメータ保存
	//UGVRS = RS0
	time_t now = time(NULL);
	struct tm* pnow = localtime(&now);
	char buff[128];
	FILE* fr;
	sprintf(buff, "D:\\Github_output\\SuperImposition\\Get_RS_IntrinsicParams\\%04d%02d%02d%02d%02d_RS0_intrinsicparams.csv", 1900 + pnow->tm_year, 1 + pnow->tm_mon, pnow->tm_mday, pnow->tm_hour, pnow->tm_min);
	fr = fopen(buff, "w");
	fprintf(fr, "%d,%d,", ugvrs_intparams.width, ugvrs_intparams.height);
	fprintf(fr, "\n");
	fprintf(fr, "%lf,%lf,", ugvrs_intparams.ppx, ugvrs_intparams.ppy);
	fprintf(fr, "\n");
	fprintf(fr, "%lf,%lf,", ugvrs_intparams.fx, ugvrs_intparams.fy);
	fprintf(fr, "\n");
	for (size_t i = 0; i < 5; i++)
	{
		fprintf(fr, "%lf,", ugvrs_intparams.coeffs[i]);
	}
	fprintf(fr, "\n");
	for (size_t i = 0; i < 9; i++)
	{
		fprintf(fr, "%lf,", ugvrs_extparams_col2dpt.rotation[i]);
	}
	fprintf(fr, "\n");
	for (size_t i = 0; i < 3; i++)
	{
		fprintf(fr, "%lf,", ugvrs_extparams_col2dpt.translation[i]);
	}
	fprintf(fr, "\n");
	fclose(fr);

	//RS1
	sprintf(buff, "D:\\Github_output\\SuperImposition\\Get_RS_IntrinsicParams\\%04d%02d%02d%02d%02d_RS1_intrinsicparams.csv", 1900 + pnow->tm_year, 1 + pnow->tm_mon, pnow->tm_mday, pnow->tm_hour, pnow->tm_min);
	fr = fopen(buff, "w");
	fprintf(fr, "%d,%d,", uavrs_intparams.width, uavrs_intparams.height);
	fprintf(fr, "\n");
	fprintf(fr, "%lf,%lf,", uavrs_intparams.ppx, uavrs_intparams.ppy);
	fprintf(fr, "\n");
	fprintf(fr, "%lf,%lf,", uavrs_intparams.fx, uavrs_intparams.fy);
	fprintf(fr, "\n");
	for (size_t i = 0; i < 5; i++)
	{
		fprintf(fr, "%lf,", uavrs_intparams.coeffs[i]);
	}
	fprintf(fr, "\n");
	for (size_t i = 0; i < 9; i++)
	{
		fprintf(fr, "%lf,", uavrs_extparams_col2dpt.rotation[i]);
	}
	fprintf(fr, "\n");
	for (size_t i = 0; i < 3; i++)
	{
		fprintf(fr, "%lf,", uavrs_extparams_col2dpt.translation[i]);
	}
	fprintf(fr, "\n");
	fclose(fr);


	return 0;
}