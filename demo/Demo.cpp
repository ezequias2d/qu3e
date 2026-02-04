//------------------------------------------------------------------------------
/**
@file	Demo.cpp

@author Randy Gaul, Ezequias Silva
@date   19/12/2025
Copyright (c) 2014 Randy Gaul http://www.randygaul.net
Copyright (c) 2025 Ezequias Silva https://github.com/ezequias2d

This software is provided 'as-is', without any express or implied
warranty. In no event will the authors be held liable for any damages
arising from the use of this software.

Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it
freely, subject to the following restrictions:
    1. The origin of this software must not be misrepresented; you must not
      claim that you wrote the original software. If you use this software
      in a product, an acknowledgment in the product documentation would be
      appreciated but is not required.
    2. Altered source versions must be plainly marked as such, and must not
      be misrepresented as being the original software.
    3. This notice may not be removed or altered from any source distribution.
*/
//------------------------------------------------------------------------------
#include "Demo.h"
#include <zabato/gpu.hpp>
#include <zabato/imgui.hpp>
#include <zabato/input.hpp>
#include <zabato/window.hpp>

#include "BoxStack.h"
#include "DropBoxes.h"
#include "DropShapes.h"
#include "RayPush.h"
#include "SphereStack.h"
#include "Test.h"

float dt = 1.0f / 60.0f;
q3Scene scene(dt);
bool paused            = false;
bool singleStep        = false;
bool enableSleep       = true;
bool enableFriction    = true;
int velocityIterations = 10;
i32 mouseX;
i32 mouseY;
bool mouseLeftDown;
bool mouseRightDown;
int windowWidth;
int windowHeight;
i32 demoCount;
i32 currentDemo;
char sceneFileName[256];
i32 lastDemo;
Demo *demos[Q3_DEMO_MAX_COUNT];

using namespace zabato;

class Renderer : public q3Render
{
public:
    void SetGPU(gpu *gpu) { m_gpu = gpu; }

    void SetPenColor(f32 r, f32 g, f32 b, f32 a = 1.0f) override
    {
        Q3_UNUSED(a);
        if (m_gpu)
            m_gpu->color(r, g, b);
    }

    void SetPenPosition(f32 x, f32 y, f32 z) override
    {
        x_ = x, y_ = y, z_ = z;
    }

    void SetScale(f32 sx, f32 sy, f32 sz) override
    {
        sx_ = sx, sy_ = sy, sz_ = sz;
    }

    void Line(f32 x, f32 y, f32 z) override
    {
        if (m_gpu)
        {
            m_gpu->enable_depth_test(false);
            m_gpu->begin(primitive_type::lines);
            m_gpu->vertex(x_, y_, z_);
            m_gpu->vertex(x, y, z);
            m_gpu->end();
            m_gpu->enable_depth_test(true);
        }
        SetPenPosition(x, y, z);
    }

    void Triangle(f32 x1,
                  f32 y1,
                  f32 z1,
                  f32 x2,
                  f32 y2,
                  f32 z2,
                  f32 x3,
                  f32 y3,
                  f32 z3) override
    {
        if (m_gpu)
        {
            m_gpu->enable_lighting(true);
            m_gpu->begin(primitive_type::triangles);
            m_gpu->normal(nx_, ny_, nz_);
            m_gpu->color(0.2f, 0.4f, 0.7f, 0.7f);
            m_gpu->vertex(x1, y1, z1);
            m_gpu->vertex(x2, y2, z2);
            m_gpu->vertex(x3, y3, z3);
            m_gpu->end();
            m_gpu->enable_lighting(false);
        }
    }

    void SetTriNormal(f32 x, f32 y, f32 z) override
    {
        nx_ = x;
        ny_ = y;
        nz_ = z;
    }

    void Point() override
    {
        if (m_gpu)
        {
            m_gpu->begin(primitive_type::points);
            m_gpu->vertex(x_, y_, z_);
            m_gpu->end();
        }
    };

    void Sphere() override
    {
        if (m_gpu)
        {
            // m_gpu->enable_depth_test(false);
            m_gpu->begin(primitive_type::lines);

            const int kSegs = 20;
            const float kPi = 3.14159265f;
            float angleStep = kPi * 2.0f / (float)kSegs;

            for (int i = 0; i < kSegs; ++i)
            {
                float a = angleStep * (float)i;
                float b = angleStep * (float)((i + 1) % kSegs);

                float ca = cos(a);
                float sa = sin(a);
                float cb = cos(b);
                float sb = sin(b);

                // XY
                m_gpu->vertex(x_ + f32(ca) * sx_, y_ + f32(sa) * sy_, z_);
                m_gpu->vertex(x_ + f32(cb) * sx_, y_ + f32(sb) * sy_, z_);

                // YZ
                m_gpu->vertex(x_, y_ + f32(ca) * sy_, z_ + f32(sa) * sz_);
                m_gpu->vertex(x_, y_ + f32(cb) * sy_, z_ + f32(sb) * sz_);

                // XZ
                m_gpu->vertex(x_ + f32(ca) * sx_, y_, z_ + f32(sa) * sz_);
                m_gpu->vertex(x_ + f32(cb) * sx_, y_, z_ + f32(sb) * sz_);
            }

            m_gpu->end();
            // m_gpu->enable_depth_test(true);
        }
    }

    void Capsule(f32 p1x,
                 f32 p1y,
                 f32 p1z,
                 f32 p2x,
                 f32 p2y,
                 f32 p2z,
                 f32 radius) override
    {
        if (m_gpu)
        {
            m_gpu->begin(primitive_type::lines);

            q3Vec3 p1(p1x, p1y, p1z);
            q3Vec3 p2(p2x, p2y, p2z);
            q3Vec3 d = p2 - p1;
            r32 len  = q3Length(d);
            if (len < 0.0001f)
                return;

            q3Vec3 y = d / len;
            q3Vec3 x, z;

            // Compute basis
            if (q3Abs(y.x) > q3Abs(y.y))
                x = q3Cross(q3Vec3(0, 1, 0), y);
            else
                x = q3Cross(q3Vec3(1, 0, 0), y);

            x = q3Normalize(x);
            z = q3Cross(x, y);

            const int kSegs = 20;
            const float kPi = 3.14159265f;
            float angleStep = kPi * 2.0f / (float)kSegs;

            for (int i = 0; i < kSegs; ++i)
            {
                float a = angleStep * (float)i;
                float b = angleStep * (float)((i + 1) % kSegs);

                float ca = cos(a);
                float sa = sin(a);
                float cb = cos(b);
                float sb = sin(b);

                // Circle at P1
                q3Vec3 v1 = p1 + (x * ca + z * sa) * radius;
                q3Vec3 v2 = p1 + (x * cb + z * sb) * radius;
                m_gpu->vertex(v1.x, v1.y, v1.z);
                m_gpu->vertex(v2.x, v2.y, v2.z);

                // Circle at P2
                q3Vec3 v3 = p2 + (x * ca + z * sa) * radius;
                q3Vec3 v4 = p2 + (x * cb + z * sb) * radius;
                m_gpu->vertex(v3.x, v3.y, v3.z);
                m_gpu->vertex(v4.x, v4.y, v4.z);

                // Connecting lines
                if (i == 0 || i == kSegs / 2 || i == kSegs / 4 ||
                    i == kSegs * 3 / 4)
                {
                    m_gpu->vertex(v1.x, v1.y, v1.z);
                    m_gpu->vertex(v3.x, v3.y, v3.z);
                }
            }

            // Draw profile arcs for hemispheres
            for (int i = 0; i < kSegs / 2; ++i)
            {
                float a = angleStep * (float)i;
                float b = angleStep * (float)(i + 1);

                // Top (P2)
                // Arc in plane defined by Y and X
                // Angle 0: Y axis (tip). Angle PI/2: X axis (side).
                // Wait, let's use standard parametric:
                // y * r * cos(theta) + x * r * sin(theta) ?
                // theta = 0 -> y*r (tip, along axis away from center P2?). Axis
                // is Y vector. P2 + y*r is the tip. P2 + x*r is side. theta 0
                // to PI.

                // Semicircle 1: P2 + radius * (y * sin(theta) + x * cos(theta))
                // -> theta 0..PI If theta=0 -> x*r (side). theta=PI/2 -> y*r
                // (tip). theta=PI -> -x*r (other side).

                // Arc 1 (in P2 + plane(x, y))
                q3Vec3 p2_v1 = p2 + x * cos(a) * radius + y * sin(a) * radius;
                q3Vec3 p2_v2 = p2 + x * cos(b) * radius + y * sin(b) * radius;
                m_gpu->vertex(p2_v1.x, p2_v1.y, p2_v1.z);
                m_gpu->vertex(p2_v2.x, p2_v2.y, p2_v2.z);

                // Arc 2 (in P2 + plane(z, y))
                q3Vec3 p2_v3 = p2 + z * cos(a) * radius + y * sin(a) * radius;
                q3Vec3 p2_v4 = p2 + z * cos(b) * radius + y * sin(b) * radius;
                m_gpu->vertex(p2_v3.x, p2_v3.y, p2_v3.z);
                m_gpu->vertex(p2_v4.x, p2_v4.y, p2_v4.z);

                // Bottom (P1)
                // Arc in plane(x, -y)
                // P1 + radius * (x * cos(theta) - y * sin(theta))

                q3Vec3 p1_v1 = p1 + x * cos(a) * radius - y * sin(a) * radius;
                q3Vec3 p1_v2 = p1 + x * cos(b) * radius - y * sin(b) * radius;
                m_gpu->vertex(p1_v1.x, p1_v1.y, p1_v1.z);
                m_gpu->vertex(p1_v2.x, p1_v2.y, p1_v2.z);

                q3Vec3 p1_v3 = p1 + z * cos(a) * radius - y * sin(a) * radius;
                q3Vec3 p1_v4 = p1 + z * cos(b) * radius - y * sin(b) * radius;
                m_gpu->vertex(p1_v3.x, p1_v3.y, p1_v3.z);
                m_gpu->vertex(p1_v4.x, p1_v4.y, p1_v4.z);
            }

            m_gpu->end();
        }
    }

private:
    gpu *m_gpu = nullptr;
    f32 x_, y_, z_;
    f32 sx_, sy_, sz_;
    f32 nx_, ny_, nz_;
};

Renderer renderer;

void OnCursorPos(window *win, real x, real y, real dx, real dy)
{
    mouseX = (int)x;
    mouseY = (int)y;
}

void OnMouseButton(window *win,
                   mouse_button button,
                   button_state state,
                   modifier_keys mods)
{

    if (state == button_state::press)
    {
        if (button == mouse_button::left)
        {
            mouseLeftDown = true;
            demos[currentDemo]->LeftClick(mouseX, mouseY);
        }
        else if (button == mouse_button::right)
        {
            mouseRightDown = true;
        }
    }
    else if (state == button_state::release)
    {
        if (button == mouse_button::left)
        {
            mouseLeftDown = false;
        }
        else if (button == mouse_button::right)
        {
            mouseRightDown = false;
        }
    }
}

namespace Camera
{
float position[3] = {0.0f, 5.0f, 20.0f};
float target[3]   = {0.0f, 0.0f, 0.0f};
}; // namespace Camera

namespace Light
{
float ambient[4]  = {1.0f, 1.0f, 1.0f, 0.5f};
float diffuse[4]  = {0.2f, 0.4f, 0.7f, 1.0f};
float specular[4] = {1.0f, 1.0f, 1.0f, 1.0f};
} // namespace Light

void OnKey(window *win, key_code key, int, button_state state, modifier_keys)
{
    using namespace zabato;
    const float increment = 0.2f;

    if (state == button_state::press || state == button_state::repeat)
    {
        if (key == key_code::escape)
            exit(0); // Exit on ESC

        switch (key)
        {
        case key_code::p:
            if (state == button_state::press)
                paused = !paused;
            break;
        case key_code::space:
            if (state == button_state::press)
            {
                paused     = true;
                singleStep = true;
            }
            break;
        case key_code::w:
            Camera::position[2] -= increment;
            Camera::target[2] -= increment;
            break;
        case key_code::s:
            Camera::position[2] += increment;
            Camera::target[2] += increment;
            break;
        case key_code::a:
            Camera::position[0] -= increment;
            Camera::target[0] -= increment;
            break;
        case key_code::d:
            Camera::position[0] += increment;
            Camera::target[0] += increment;
            break;
        case key_code::q:
            Camera::position[1] -= increment;
            Camera::target[1] -= increment;
            break;
        case key_code::e:
            Camera::position[1] += increment;
            Camera::target[1] += increment;
            break;
        default:
            break;
        }

        if (key != key_code::unknown)
            demos[currentDemo]->KeyDown(key);
    }
    else if (state == button_state::release)
    {
        if (key != key_code::unknown)
            demos[currentDemo]->KeyUp(key);
    }
}

texture *g_whiteTex = nullptr;

void RenderFrame(window *win, gpu *gpu)
{
    gpu->new_frame();
    gpu->enable_scissor_test(false);

    if (g_whiteTex)
        gpu->bind_texture(g_whiteTex);
    else
        gpu->unbind_texture();

    vec2<int> size = win->get_framebuffer_size();
    int w          = size.x;
    int h          = size.y;
    if (h <= 0)
        h = 1;

    f32 aspectRatio = (f32)w / (f32)h;
    gpu->viewport(w, h);
    gpu->set_matrix_mode(matrix_mode::projection);
    gpu->load_identity();
    gpu->perspective_fov(
        45.0f * (3.14159f / 180.0f), aspectRatio, 0.1f, 10000.0f);
    gpu->set_matrix_mode(matrix_mode::modelview);
    gpu->load_identity();

    vec3<real> pos(
        Camera::position[0], Camera::position[1], Camera::position[2]);
    vec3<real> target(Camera::target[0], Camera::target[1], Camera::target[2]);
    vec3<real> up(0.0f, 1.0f, 0.0f);
    mat4<real> view = mat4_look_at(pos, target, up);
    gpu->load_matrix(view);

    imgui::new_frame();

    ImGui::SetNextWindowPos(ImVec2(float(w - 300 - 30), 30),
                            ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(300, 225), ImGuiCond_FirstUseEver);
    ImGui::Begin("q3Scene Settings", NULL, 0);
    ImGui::Combo("Demo",
                 &currentDemo,
                 "Drop Boxes\0Ray Push\0Box Stack\0Test\0Sphere Stack\0Drop "
                 "Shapes\0Height Map\0");
    ImGui::Checkbox("Pause", &paused);
    if (paused)
        ImGui::Checkbox("Single Step", &singleStep);
    ImGui::Checkbox("Sleeping", &enableSleep);
    ImGui::Checkbox("Friction", &enableFriction);
    ImGui::SliderInt("Iterations", &velocityIterations, 1, 50);
    int flags = (1 << 0) | (1 << 1) | (1 << 2);
    ImGui::InputText("Dump File Name",
                     sceneFileName,
                     ((int)(sizeof(sceneFileName) / sizeof(*sceneFileName))),
                     flags);
    if (ImGui::Button("Dump Scene"))
    {
        FILE *fp = fopen(sceneFileName, "w");
        scene.Dump(fp);
        fclose(fp);
    }
    ImGui::End();

    // Switch demo if needed
    if (currentDemo != lastDemo)
    {
        demos[lastDemo]->Shutdown();
        demos[currentDemo]->Init();
        lastDemo = currentDemo;
    }

    gpu->clear({0.0f, 0.0f, 0.0f, 0.0f}, 1.0f);

    scene.Render(&renderer);

    demos[currentDemo]->Render(&renderer);

    ImGui::Render();
    imgui::render_draw_data(ImGui::GetDrawData());

    win->swap_buffers();
}

void InitDemo(window *win, gpu *gpu)
{
    renderer.SetGPU(gpu);

    if (!g_whiteTex)
    {
        g_whiteTex     = gpu->create_texture(1, 1, color_format::rgba4444);
        uint32_t white = 0xFFFF;
        g_whiteTex->load(1, 1, color_format::rgba4444, sizeof(white), &white);
    }

    gpu->enable_depth_test(true);
    gpu->enable_blend(true);
    gpu->set_blend_func(blend_factor::src_alpha,
                        blend_factor::one_minus_src_alpha);

    light_data l;
    l.type     = light_type::point;
    l.ambient  = {Light::ambient[0],
                  Light::ambient[1],
                  Light::ambient[2],
                  Light::ambient[3]};
    l.diffuse  = {Light::diffuse[0],
                  Light::diffuse[1],
                  Light::diffuse[2],
                  Light::diffuse[3]};
    l.specular = {Light::specular[0],
                  Light::specular[1],
                  Light::specular[2],
                  Light::specular[3]};
    l.position = {
        Camera::position[0], Camera::position[1], Camera::position[2]};

    l.spot_cutoff           = 180.0f;
    l.constant_attenuation  = 1.0f;
    l.linear_attenuation    = 0.0f;
    l.quadratic_attenuation = 0.0f;

    gpu->set_light(0, &l);
    gpu->enable_lighting(true);

    demos[0]    = new DropBoxes();
    demos[1]    = new RayPush();
    demos[2]    = new BoxStack();
    demos[3]    = new Test();
    demos[4]    = new SphereStack();
    demos[5]    = new DropShapes();
    demoCount   = 6;
    currentDemo = 5;
    demos[currentDemo]->Init();
    sprintf(sceneFileName, "q3dump.txt");

    win->add_key_callback(OnKey);
    win->add_mouse_button_callback(OnMouseButton);
    win->add_cursor_move_callback(OnCursorPos);
}

void UpdateFrame(float time)
{
    // Set scene settings
    scene.SetAllowSleep(enableSleep);
    scene.SetEnableFriction(enableFriction);
    scene.SetIterations(velocityIterations);

    // Physics Step Logic
    static f32 accumulator = 0;
    accumulator += time;

    accumulator = clamp(accumulator, f32(0), f32(1));
    while (accumulator >= dt)
    {
        if (!paused)
        {
            scene.Step();
            demos[currentDemo]->Update();
        }

        else
        {
            if (singleStep)
            {
                scene.Step();
                demos[currentDemo]->Update();
                singleStep = false;
            }
        }

        accumulator -= dt;
    }
}
