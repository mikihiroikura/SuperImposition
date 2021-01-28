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

const int vert_cnt = 407040;
const int realsense_cnt = 1;

extern void initGL();
extern void drawGL_realsense(float* pts0, int* pc0_ringid, float* pc0_texcoords);
extern void finishGL();

#endif // !GRAPHICS_H_
