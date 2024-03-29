#pragma once
#ifndef GRAPHICS_H_
#define GRAPHICS_H_

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtc/matrix_transform.hpp> 
#include <glm/gtx/transform.hpp>

// imgui include
#include "imgui/imgui.h"
#include "imgui/imgui_impl_opengl3.h"
#include "imgui/imgui_impl_glfw.h"

//realsense include
#include <librealsense2/rs.hpp>
#include "realsense.h"

//realsense
const unsigned int colorwidth = 1920;
const unsigned int colorheight = 1080;
const unsigned int colorfps = 30;
const unsigned int depthwidth = 1280;
const unsigned int depthheight = 720;
const unsigned int depthfps = 30;

const int vert_cnt = depthwidth * depthheight;
const int realsense_cnt = 2;

extern void initGL();
extern void drawGL_realsense(float** pts0, float** pc0_texcoords, rs2::frame** colorframes);
extern void finishGL();

#endif // !GRAPHICS_H_
