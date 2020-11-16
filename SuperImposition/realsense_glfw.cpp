#include "realsense_glfw.h"
#include "../third-party/stb_easy_font.h"

//const rs2::vertex* vertices;
//const rs2::texture_coordinate* tex_coords;


inline void draw_text(int x, int y, const char* text)
{
    char buffer[60000]; // ~300 chars
    glEnableClientState(GL_VERTEX_ARRAY);
    glVertexPointer(2, GL_FLOAT, 16, buffer);
    glDrawArrays(GL_QUADS, 0, 4 * stb_easy_font_print((float)x, (float)(y - 7), (char*)text, nullptr, buffer, sizeof(buffer)));
    glDisableClientState(GL_VERTEX_ARRAY);
}

void set_viewport(const rect& r)
{
    glViewport((int)r.x, (int)r.y, (int)r.w, (int)r.h);
    glLoadIdentity();
    glMatrixMode(GL_PROJECTION);
    glOrtho(0, r.w, r.h, 0, -1, +1);
}

// Handles all the OpenGL calls needed to display the point cloud
void draw_pointcloud(const rs2::vertex* vertices, GLuint* vbo, const rs2::texture_coordinate* tex_coords, GLuint* tcbo, float width, float height, texture_gl& tex, rs2::points& points, float translate_z, float rotate_x, float rotate_y)
{
    if (!points)
        return;

    // OpenGL commands that prep screen for the pointcloud
    /*glLoadIdentity();*/
    glPushAttrib(GL_ALL_ATTRIB_BITS);

    glClearColor(153.f / 255, 153.f / 255, 153.f / 255, 1);
    glClear(GL_DEPTH_BUFFER_BIT);

    //glMatrixMode(GL_PROJECTION);
    //glPushMatrix();
    //gluPerspective(60, width / height, 0.01f, 10.0f);

    /*glMatrixMode(GL_MODELVIEW);*/
    //glPushMatrix();
    //gluLookAt(0, 0, 0, 0, 0, 1, 0, -1, 0); //Ç±ÇÍÇ≈ÉJÉÅÉâÇÃè„å¸Ç´ÇÃé≤Ç--yï˚å¸Ç…Ç∑ÇÈÇ±Ç∆Ç≈è„â∫ÇçáÇÌÇπÇÈ

    /*glRotated(rotate_x, 1, 0, 0);
    glRotated(rotate_y, 0, 1, 0);
    glTranslatef(0, 0, translate_z);*/

    glPointSize(width / 640);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, tex.get_gl_handle());
    float tex_border_color[] = { 0.8f, 0.8f, 0.8f, 0.8f };
    glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, tex_border_color);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, 0x812F); // GL_CLAMP_TO_EDGE
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, 0x812F); // GL_CLAMP_TO_EDGE


    /* this segment actually prints the pointcloud */
    vertices = points.get_vertices();              // get vertices
    tex_coords = points.get_texture_coordinates(); // and texture coordinates
    glBindBuffer(GL_ARRAY_BUFFER, *vbo);
    glBufferData(GL_ARRAY_BUFFER, 407040 * 3 * sizeof(float), vertices, GL_DYNAMIC_DRAW);
    
    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_TEXTURE_COORD_ARRAY);
    glVertexPointer(3, GL_FLOAT, 0, 0);
    glBindBuffer(GL_ARRAY_BUFFER, *tcbo);
    glBufferData(GL_ARRAY_BUFFER, 407040 * 2 * sizeof(float), tex_coords, GL_DYNAMIC_DRAW);
    glTexCoordPointer(2, GL_FLOAT, 0, 0);
    glDrawArrays(GL_POINTS, 0, 407040);

    glDisableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_TEXTURE_COORD_ARRAY);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    


    // OpenGL cleanup
    /*glPopMatrix();*/
    //glMatrixMode(GL_PROJECTION);
    //glPopMatrix();
    glPopAttrib();
}

// Handles all the OpenGL calls needed to display the point cloud
void draw_pointcloud2(GLuint* vbo, const rs2::texture_coordinate* tex_coords, GLuint* tcbo, float width, float height, texture_gl& tex, rs2::points& points, float translate_z, float rotate_x, float rotate_y)
{
    if (!points)
        return;

    // OpenGL commands that prep screen for the pointcloud
    /*glLoadIdentity();*/
    glPushAttrib(GL_ALL_ATTRIB_BITS);

    glClearColor(153.f / 255, 153.f / 255, 153.f / 255, 1);
    glClear(GL_DEPTH_BUFFER_BIT);

    glPointSize(width / 640);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, tex.get_gl_handle());
    float tex_border_color[] = { 0.8f, 0.8f, 0.8f, 0.8f };
    glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, tex_border_color);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, 0x812F); // GL_CLAMP_TO_EDGE
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, 0x812F); // GL_CLAMP_TO_EDGE


    /* this segment actually prints the pointcloud */
    tex_coords = points.get_texture_coordinates(); // and texture coordinates
    glBindBuffer(GL_ARRAY_BUFFER, *vbo);

    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_TEXTURE_COORD_ARRAY);
    glVertexPointer(3, GL_FLOAT, 0, 0);
    glBindBuffer(GL_ARRAY_BUFFER, *tcbo);
    glBufferData(GL_ARRAY_BUFFER, 407040 * 2 * sizeof(float), tex_coords, GL_DYNAMIC_DRAW);
    glTexCoordPointer(2, GL_FLOAT, 0, 0);
    glDrawArrays(GL_POINTS, 0, 407040);

    glDisableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_TEXTURE_COORD_ARRAY);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    // OpenGL cleanup
    glPopAttrib();
}

void quat2mat(rs2_quaternion& q, GLfloat H[16])  // to column-major matrix
{
    H[0] = 1 - 2 * q.y * q.y - 2 * q.z * q.z; H[4] = 2 * q.x * q.y - 2 * q.z * q.w;     H[8] = 2 * q.x * q.z + 2 * q.y * q.w;     H[12] = 0.0f;
    H[1] = 2 * q.x * q.y + 2 * q.z * q.w;     H[5] = 1 - 2 * q.x * q.x - 2 * q.z * q.z; H[9] = 2 * q.y * q.z - 2 * q.x * q.w;     H[13] = 0.0f;
    H[2] = 2 * q.x * q.z - 2 * q.y * q.w;     H[6] = 2 * q.y * q.z + 2 * q.x * q.w;     H[10] = 1 - 2 * q.x * q.x - 2 * q.y * q.y; H[14] = 0.0f;
    H[3] = 0.0f;                      H[7] = 0.0f;                      H[11] = 0.0f;                      H[15] = 1.0f;
}

// Handles all the OpenGL calls needed to display the point cloud w.r.t. static reference frame
void draw_pointcloud_wrt_world(float width, float height, glfw_state& app_state, rs2::points& points, rs2_pose& pose, float H_t265_d400[16], std::vector<rs2_vector>& trajectory)
{
    if (!points)
        return;

    // OpenGL commands that prep screen for the pointcloud
    glLoadIdentity();
    glPushAttrib(GL_ALL_ATTRIB_BITS);

    glClearColor(153.f / 255, 153.f / 255, 153.f / 255, 1);
    glClear(GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    gluPerspective(60, width / height, 0.01f, 10.0f);


    // viewing matrix
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();

    // rotated from depth to world frame: z => -z, y => -y
    glTranslatef(0, 0, -0.75f - app_state.offset_y * 0.05f);
    glRotated(app_state.roll, 1, 0, 0);
    glRotated(app_state.pitch, 0, -1, 0);
    glTranslatef(0, 0, 0.5f);

    // draw trajectory
    glEnable(GL_DEPTH_TEST);
    glLineWidth(2.0f);
    glBegin(GL_LINE_STRIP);
    for (auto&& v : trajectory)
    {
        glColor3f(0.0f, 1.0f, 0.0f);
        glVertex3f(v.x, v.y, v.z);
    }
    glEnd();
    glLineWidth(0.5f);
    glColor3f(1.0f, 1.0f, 1.0f);

    // T265 pose
    GLfloat H_world_t265[16];
    quat2mat(pose.rotation, H_world_t265);
    H_world_t265[12] = pose.translation.x;
    H_world_t265[13] = pose.translation.y;
    H_world_t265[14] = pose.translation.z;

    glMultMatrixf(H_world_t265);

    // T265 to D4xx extrinsics
    glMultMatrixf(H_t265_d400);


    glPointSize(width / 640);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, app_state.tex.get_gl_handle());
    float tex_border_color[] = { 0.8f, 0.8f, 0.8f, 0.8f };
    glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, tex_border_color);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, 0x812F); // GL_CLAMP_TO_EDGE
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, 0x812F); // GL_CLAMP_TO_EDGE
    glBegin(GL_POINTS);

    /* this segment actually prints the pointcloud */
    auto vertices = points.get_vertices();              // get vertices
    auto tex_coords = points.get_texture_coordinates(); // and texture coordinates
    for (int i = 0; i < points.size(); i++)
    {
        if (vertices[i].z)
        {
            // upload the point and texture coordinates only for points we have depth data for
            glVertex3fv(vertices[i]);
            glTexCoord2fv(tex_coords[i]);
        }
    }

    // OpenGL cleanup
    glEnd();
    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glPopAttrib();
}

void window::close()
{
    glfwSetWindowShouldClose(win, 1);
}