#pragma once
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

#include <string>

using namespace std;

class realsense
{
private:
	
	rs2::frameset frames;
	rs2::colorizer color_map;
	rs2::pointcloud pc;

	//Color img
	unsigned int color_width = 1920;
	unsigned int color_height = 1080;
	unsigned int color_fps = 30;

	//Depth img
	unsigned int depth_width = 1280;
	unsigned int depth_height = 720;
	unsigned int depth_fps = 30;

public:
	rs2::pipeline pipe;
	rs2::frame colorframe;
	rs2::frame depthframe;
	rs2::frame depthframe_filter;
	cv::Mat colorimg;
	cv::Mat depthimg;
	rs2::points points;
	string serial_num;
	
	realsense(string serial_num);
	realsense(string num, rs2_format colorformat, rs2_format depthformat);
	realsense(string num, rs2_format colorformat, unsigned int color_width, unsigned int color_height, unsigned int color_fps, rs2_format depthformat, unsigned int depth_width, unsigned int depth_height, unsigned int depth_fps);

	void update_frame();
	void update_color();
	void transform_color_img();
	void update_depth();
	void transform_depth_img();
	void calc_pointcloud();
};

