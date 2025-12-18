add_rules("mode.debug", "mode.release")
add_rules("plugin.compile_commands.autoupdate", {outputdir = ".vscode"})

add_requires("freeglut")

target("qu3e")
    set_kind("static")
    set_languages("c++11")
    add_files("src/**.cpp") 
    add_includedirs("src", {public = true})
    add_headerfiles("src/**.h", "src/**.inl")

-- target("freeglut") removed, using system package

target("imgui_qu3e")
    set_kind("static")
    set_languages("c++11")
    add_files("imgui/**.cpp")
    add_includedirs("imgui", {public = true})

target("demo")
    set_kind("binary")
    set_languages("c++11")
    add_files("demo/**.cpp")
    add_includedirs("demo")
    add_deps("qu3e", "imgui_qu3e") -- freeglut is a package now
    add_packages("freeglut")
    add_links("GL", "GLU")
