#include "Demo.h"
#include <zabato/fs.hpp>
#include <zabato/gpu.hpp>
#include <zabato/imgui.hpp>
#include <zabato/window.hpp>

int main(int argc, char **argv)
{
    if (!init_window_system())
        return 1;

    window *win = create_window(
        100, 100, 1000, 600, "qu3e Physics by Randy Gaul", window_flags::none);

    if (!win)
        return 1;

    make_context_current(win);

    gpu *gpu = init_gpu();
    if (!gpu)
        return 1;

    fs::virtual_fs fs;
    imgui::init(win, fs);

    InitDemo(win, gpu);

    uint64_t last_time = get_time();

    while (!win->should_close())
    {
        poll_events();

        uint64_t current_time = get_time();
        float dt              = (float)(current_time - last_time) / 1000.0f;
        last_time             = current_time;

        if (dt > 0.1f)
            dt = 0.1f;

        UpdateFrame(dt);

        RenderFrame(win, gpu);
    }

    imgui::shutdown();
    terminate_window_system();

    return 0;
}
