#include "Demo.h"
#include <zabato/gpu.hpp>
#include <zabato/imgui.hpp>
#include <zabato/window.hpp>

int main(int argc, char **argv)
{
    if (!zabato::init_window_system())
        return 1;

    zabato::window *win = zabato::create_window(100,
                                                100,
                                                1000,
                                                600,
                                                "qu3e Physics by Randy Gaul",
                                                zabato::window_flags::none);

    if (!win)
        return 1;

    zabato::make_context_current(win);

    zabato::gpu *gpu = zabato::init_gpu();
    if (!gpu)
        return 1;

    zabato::imgui::init(win);

    InitDemo(win, gpu);

    uint64_t last_time = zabato::get_time();

    while (!win->should_close())
    {
        zabato::poll_events();

        uint64_t current_time = zabato::get_time();
        float dt              = (float)(current_time - last_time) / 1000.0f;
        last_time             = current_time;

        if (dt > 0.1f)
            dt = 0.1f;

        UpdateFrame(dt);

        RenderFrame(win, gpu);
    }

    zabato::imgui::shutdown();
    zabato::terminate_window_system();

    return 0;
}
