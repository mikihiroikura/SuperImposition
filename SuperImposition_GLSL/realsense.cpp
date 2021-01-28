#include "realsense.h"

realsense::realsense(string num) {
	rs2::config config;
	realsense::serial_num = num;
	config.enable_device(realsense::serial_num);
	config.enable_stream(RS2_STREAM_COLOR, realsense::color_width, realsense::color_height, RS2_FORMAT_BGR8, realsense::color_fps);
	config.enable_stream(RS2_STREAM_DEPTH, realsense::depth_width, realsense::depth_height, RS2_FORMAT_Z16, realsense::depth_fps);

	realsense::pipe.start(config);
}

realsense::realsense(string num, rs2_format colorformat, rs2_format depthformat) {
	rs2::config config;
	realsense::serial_num = num;
	config.enable_device(realsense::serial_num);
	config.enable_stream(RS2_STREAM_COLOR, realsense::color_width, realsense::color_height, colorformat, realsense::color_fps);
	config.enable_stream(RS2_STREAM_DEPTH, realsense::depth_width, realsense::depth_height, depthformat, realsense::depth_fps);

	realsense::pipe.start(config);
}

void realsense::update_frame() {
	realsense::frames = realsense::pipe.wait_for_frames();
}

void realsense::update_color() {
	realsense::colorframe = realsense::frames.get_color_frame();
}

void realsense::transform_color_img() {
	realsense::colorimg = cv::Mat(cv::Size(realsense::color_width, realsense::color_height), CV_8UC3, (void*)realsense::colorframe.get_data(), cv::Mat::AUTO_STEP);
}

void realsense::update_depth() {
	realsense::depthframe = realsense::frames.get_depth_frame();
}

void realsense::transform_depth_img() {
	realsense::depthframe_filter = realsense::depthframe.apply_filter(color_map);
	realsense::depthimg = cv::Mat(cv::Size(realsense::depth_width, realsense::depth_height), CV_8UC3, (void*)realsense::depthframe_filter.get_data(), cv::Mat::AUTO_STEP);
}

void realsense::calc_pointcloud() {
	pc.map_to(realsense::colorframe);
	points = pc.calculate(realsense::depthframe);
}