//------------------------------------------------------------------------------
/**
@file	Demo.cpp

@author	Randy Gaul
@date	11/25/2014
Copyright (c) 2014 Randy Gaul http://www.randygaul.net

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
#include "RayPush.h"
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

class Renderer : public q3Render
{
public:
    void SetGPU(zabato::gpu *gpu) { m_gpu = gpu; }

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
            m_gpu->begin(zabato::primitive_type::lines);
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
            m_gpu->begin(zabato::primitive_type::triangles);
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
            m_gpu->begin(zabato::primitive_type::points);
            m_gpu->vertex(x_, y_, z_);
            m_gpu->end();
        }
    };

private:
    zabato::gpu *m_gpu = nullptr;
    f32 x_, y_, z_;
    f32 sx_, sy_, sz_;
    f32 nx_, ny_, nz_;
};

Renderer renderer;

void OnCursorPos(zabato::window *win, zabato::real x, zabato::real y)
{
    mouseX = (int)x;
    mouseY = (int)y;
}

void OnMouseButton(zabato::window *win,
                   zabato::mouse_button button,
                   zabato::button_state state,
                   zabato::modifier_keys mods)
{
    using namespace zabato;
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

void OnKey(zabato::window *win,
           zabato::key_code key,
           int,
           zabato::button_state state,
           zabato::modifier_keys)
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

zabato::texture *g_whiteTex = nullptr;

void RenderFrame(zabato::window *win, zabato::gpu *gpu)
{
    gpu->new_frame();
    gpu->enable_scissor_test(false);

    if (g_whiteTex)
        gpu->bind_texture(g_whiteTex);
    else
        gpu->unbind_texture();

    zabato::vec2<int> size = win->get_framebuffer_size();
    int w                  = size.x;
    int h                  = size.y;
    if (h <= 0)
        h = 1;

    f32 aspectRatio = (f32)w / (f32)h;
    gpu->viewport(w, h);
    gpu->set_matrix_mode(zabato::matrix_mode::projection);
    gpu->load_identity();
    gpu->perspective_fov(
        45.0f * (3.14159f / 180.0f), aspectRatio, 0.1f, 10000.0f);
    gpu->set_matrix_mode(zabato::matrix_mode::modelview);
    gpu->load_identity();

    zabato::vec3<zabato::real> pos(
        Camera::position[0], Camera::position[1], Camera::position[2]);
    zabato::vec3<zabato::real> target(
        Camera::target[0], Camera::target[1], Camera::target[2]);
    zabato::vec3<zabato::real> up(0.0f, 1.0f, 0.0f);
    zabato::mat4<zabato::real> view = zabato::mat4_look_at(pos, target, up);
    gpu->load_matrix(view);

    zabato::imgui::new_frame();

    ImGui::SetNextWindowPos(ImVec2(float(w - 300 - 30), 30),
                            ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(300, 225), ImGuiCond_FirstUseEver);
    ImGui::Begin("q3Scene Settings", NULL, 0);
    ImGui::Combo(
        "Demo", &currentDemo, "Drop Boxes\0Ray Push\0Box Stack\0Test\0");
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
    zabato::imgui::render_draw_data(ImGui::GetDrawData());

    win->swap_buffers();
}

void InitDemo(zabato::window *win, zabato::gpu *gpu)
{
    renderer.SetGPU(gpu);

    if (!g_whiteTex)
    {
        g_whiteTex = gpu->create_texture(1, 1, zabato::color_format::rgba4444);
        uint32_t white = 0xFFFF;
        g_whiteTex->load(
            1, 1, zabato::color_format::rgba4444, sizeof(white), &white);
    }

    gpu->enable_depth_test(true);
    gpu->enable_blend(true);
    gpu->set_blend_func(zabato::blend_factor::src_alpha,
                        zabato::blend_factor::one_minus_src_alpha);

    zabato::light l;
    l.type     = zabato::light_type::point;
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
    demoCount   = 4;
    currentDemo = 3;
    demos[currentDemo]->Init();
    sprintf(sceneFileName, "q3dump.txt");

    win->add_key_callback(OnKey);
    win->add_mouse_button_callback(OnMouseButton);
    win->add_cursor_pos_callback(OnCursorPos);
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

    accumulator = q3Clamp01(accumulator);
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
