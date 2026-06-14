^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package iridescence
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* 1.0.1
* add set_colormap_range
* Contributors: k.koide

1.0.0 (2026-01-03)
------------------
* v1.0.0 (`#188 <https://github.com/koide3/iridescence/issues/188>`_)
  * add numpy interfaces
  * more imgui bindings
  * return viewer shader setting reference
  * refactor shader setting
  * vertex colormap
  * move camera along z
  * fix vertex color
  * replace shared_ptr with unique_ptr
  * rapidhash
  * replace string name with hash name in glsl_shader
  * flat shader parameters
  * refactor and warn
  * tune ssao intensity
  * remove Eigen::aligned_allocator and fix array uniform bug
  * support int and float arrays
  * add fullscreen
  * hovered and cmap
  * fix bugs
  * install stubs
  * update README
  * update docs
  * remove link to doc
  * doc
  * include global funcs
  * doc ci
  * add links to API docs
  * trigger on only tags
  * fix ppa
  * imgui windows
  * add rainbow settings
  * japanese docs
  * fix site url
  * yodan
  * add image
  * fix typo
  * docs (wip)
  * tweak ssao params
  * docs
  * revert light viewer constructors to public
  * v1.0.0
* add colors (`#192 <https://github.com/koide3/iridescence/issues/192>`_)
* Add color (`#189 <https://github.com/koide3/iridescence/issues/189>`_)
  * add_color for 3f array
  * add colors
  * save / load camera
  * save color / depth buffers
  * set_point_shape
  * add point shape/size utils in viewer menu
  * it was too dark
* invoke_once (`#190 <https://github.com/koide3/iridescence/issues/190>`_)
* fix add_color (`#191 <https://github.com/koide3/iridescence/issues/191>`_)
* Fixed shader path issue (`#183 <https://github.com/koide3/iridescence/issues/183>`_)
  * Fixed shader path issue
  * Added support for windows
* Contributors: Harshal Deshpande, koide3

0.1.9 (2025-12-23)
------------------
* soname (`#187 <https://github.com/koide3/iridescence/issues/187>`_)
* Contributors: koide3

0.1.8 (2025-12-21 16:04)
------------------------
* Publish on PyPI (`#186 <https://github.com/koide3/iridescence/issues/186>`_)
* Contributors: koide3

0.1.7 (2025-12-21 00:55)
------------------------
* add ColorEdit4 (`#185 <https://github.com/koide3/iridescence/issues/185>`_)
* add publish.yml (`#182 <https://github.com/koide3/iridescence/issues/182>`_)
  * add publish.yml
  * fix python versions
  * install dependencies
  * recursive submodule clone
  * change ci name
  * publish
  * project info
  * fix licence
  * fix pyproject.toml
  * fix setup.py
  * publish only tags
  * fix pyridescence_data
  * fix
  * depedency on numpy
  * remove scipy
* ci on win (`#184 <https://github.com/koide3/iridescence/issues/184>`_)
* This PR fixes a file descriptor leak in the GPU monitoring module. (`#181 <https://github.com/koide3/iridescence/issues/181>`_)
  The code uses popen() to query GPU utilization but does not call pclose().
  As a result, each update cycle leaks one file descriptor. After long-running
  sessions, the process reaches the FD limit (1024), causing failures in popen(),
  portable-file-dialogs, and UI features relying on fork/pipe.
  This patch adds the missing pclose() and ensures proper cleanup.
  Tested: FD count remains stable during long runtime after the fix.
* Adds import to fix cassert import error (`#178 <https://github.com/koide3/iridescence/issues/178>`_)
* Respect window in pick_depth() (`#177 <https://github.com/koide3/iridescence/issues/177>`_)
* memory usage for point cloud buffer (`#176 <https://github.com/koide3/iridescence/issues/176>`_)
* quit cleanly by calling destory explicitly (`#174 <https://github.com/koide3/iridescence/issues/174>`_)
* fixed installation command from source (`#173 <https://github.com/koide3/iridescence/issues/173>`_)
* update_drawable_setting (`#171 <https://github.com/koide3/iridescence/issues/171>`_)
* ply io for py (`#170 <https://github.com/koide3/iridescence/issues/170>`_)
* add python interfaces for viewers (`#169 <https://github.com/koide3/iridescence/issues/169>`_)
  * add python interfaces for viewers
  * add fit_plot
* fix path for async viewer in python (`#168 <https://github.com/koide3/iridescence/issues/168>`_)
* Contributors: Aquablue, Rasmus, Vir Shah, atinfinity, koide3

0.1.6 (2025-04-23)
------------------
* add disable_partial_rendering (`#167 <https://github.com/koide3/iridescence/issues/167>`_)
* Contributors: koide3

0.1.5 (2025-04-11)
------------------
* Fix `find_sub_viewer` (`#165 <https://github.com/koide3/iridescence/issues/165>`_)
* accepts more int types (`#164 <https://github.com/koide3/iridescence/issues/164>`_)
* Generic PLY IO (`#163 <https://github.com/koide3/iridescence/issues/163>`_)
  * improve type convertion speed with batched operation
  * generic ply io
  * test for PLY IO
  * uncomment
  * enable double test
  * comment handling
  * remove offsets from PLYData
  * add_prop
* set build tool to CMake and add rosdep keys for dependencies (`#162 <https://github.com/koide3/iridescence/issues/162>`_)
* improve type convertion speed with batched operation (`#161 <https://github.com/koide3/iridescence/issues/161>`_)
* Contributors: Christian Rauch, Vincent Richard, koide3

0.1.4 (2025-02-06)
------------------
* fix wrong color attachments in FrameBuffer::set_size (`#160 <https://github.com/koide3/iridescence/issues/160>`_)
* fix typo (`#159 <https://github.com/koide3/iridescence/issues/159>`_)
* add force kill (`#158 <https://github.com/koide3/iridescence/issues/158>`_)
* plot group order (`#156 <https://github.com/koide3/iridescence/issues/156>`_)
* Plot (`#148 <https://github.com/koide3/iridescence/issues/148>`_)
  * turn table
  * set_line_width for py
  * fix plot transform
  * setup legend
  * link axes
  * fix update_plot_line
  * adaptor for 1D data
  * add ninja
  * fix update_plot_line declaration
  * additional interfaces for update_plot_scatter
  * fill shader setting interface
  * html colors
  * fix regression
  * fix examples
  * add non-template translate()
  * initializer_list constructor for FlatColor
  * color_mode
  * fix scale for scalar
  * step execute for toggle_spin
  * accepts vector3d for lookat
  * fix lookat for python
  * fix lookat variants
* add ninja (`#155 <https://github.com/koide3/iridescence/issues/155>`_)
* look for data at install location (`#154 <https://github.com/koide3/iridescence/issues/154>`_)
  When the package is installed at a non-standard location, i.e. with
  prefix different from /usr or /usr/local, the data directory cannot be
  found. This change ensures that the install location is always tried
  when looking for the data directory.
* fix depth option for capture example (`#147 <https://github.com/koide3/iridescence/issues/147>`_)
  * fix depth option for capture example
  * warn if num_elements does not match texture format
* make PCL optional (`#144 <https://github.com/koide3/iridescence/issues/144>`_)
* add PCL inclusion if want to build the pointcloud example (`#143 <https://github.com/koide3/iridescence/issues/143>`_)
  * fix set_pose typo in fps_camera_control
  * update PCL checking, before adding, force find pcl will cause error for inclusion
  * turn of build examples
  * change from PCL setup
* removing hovered drawings (`#135 <https://github.com/koide3/iridescence/issues/135>`_)
* Contributors: Michal Sojka, Seekerzero, koide3

0.1.3 (2024-08-21)
------------------
* add running (`#134 <https://github.com/koide3/iridescence/issues/134>`_)
* [update] Add definition of 'frustum/wire_frustum' in Python API (`#133 <https://github.com/koide3/iridescence/issues/133>`_)
* Contributors: ShigemichiMatsuzaki, koide3

0.1.2 (2024-08-05)
------------------
* debian package options (`#131 <https://github.com/koide3/iridescence/issues/131>`_)
* Fixed set_pose in fps camera control (`#132 <https://github.com/koide3/iridescence/issues/132>`_)
* add libglfw3-dev as dependency
* Contributors: Patiphon Narksri, k.koide, koide3

0.1.1 (2024-07-23)
------------------
* add ppa installation (`#130 <https://github.com/koide3/iridescence/issues/130>`_)
* Update README.md
* add Dockerfile_deb
* debian package options (`#129 <https://github.com/koide3/iridescence/issues/129>`_)
* generate deb (`#127 <https://github.com/koide3/iridescence/issues/127>`_)
* Trigger (`#125 <https://github.com/koide3/iridescence/issues/125>`_)
  * trigger gtsam_docker
  * remove needs
  * remove trigger branch
* add color-related shader setting util (`#124 <https://github.com/koide3/iridescence/issues/124>`_)
  * add color-related shader setting util
  * docs
* hide glm (`#123 <https://github.com/koide3/iridescence/issues/123>`_)
* switch to scikit-build (`#121 <https://github.com/koide3/iridescence/issues/121>`_)
  * switch to scikit-build
  * update installation instructions
  * update docker
  * reform constructors of Lines
  * line constructors
  * test
  * fix rotate
  * CI for test
  * fix test
  * force run xvfb
  * modernize cmake
  * fix installation
  * fix ci errors
  * windows
  * fix dl and pthread link
  * fix export macro
  * fix link errors
* minimize invoke request queue lock (`#122 <https://github.com/koide3/iridescence/issues/122>`_)
* remove boost dependency (`#110 <https://github.com/koide3/iridescence/issues/110>`_)
  * remove boost dependency
  * remove pthread
  * add pthread
  * use static lib for python bindings
  * update pybind11
  * fix python bindings
  * use local pybind11
  * remove dependency on fmt
  * remove fmt contd
  * link to fmt for spdlog testing
  * improve installation
  * Uppercase
* Partial (`#120 <https://github.com/koide3/iridescence/issues/120>`_)
  * partial rendering example
  * doc for partial
  * add note
* partial rendering example (`#119 <https://github.com/koide3/iridescence/issues/119>`_)
  * partial rendering example
  * doc for partial
* fix add_color (`#115 <https://github.com/koide3/iridescence/issues/115>`_)
* disable_vsync (`#108 <https://github.com/koide3/iridescence/issues/108>`_)
  * disable_vsync
  * add matrix constructors
* png_io.cpp: fix libpng include (`#105 <https://github.com/koide3/iridescence/issues/105>`_)
  libpng headers do not include a directory prefix. This only accidentally worked with system headers.
* ADD grid_color option (`#104 <https://github.com/koide3/iridescence/issues/104>`_)
  * WIP
  * ADD set_color method
  * WIP pass the build
  * Fix format
  * REMOVE set_color in texture and CHG update_color
* remove_ui_callback (`#102 <https://github.com/koide3/iridescence/issues/102>`_)
  * remove_ui_callback
  * remove_drawable_filter
* Guizmo fine control (`#101 <https://github.com/koide3/iridescence/issues/101>`_)
  * add set_gizmo_operation
  * enable/disable guizmo
  * set_gizmo_mode
  * update ImGuizmo and add set_clip_space
  * fix pybind
* reset to default line width (`#99 <https://github.com/koide3/iridescence/issues/99>`_)
* fix set_pose typo in fps_camera_control (`#97 <https://github.com/koide3/iridescence/issues/97>`_)
* set nullptr for default aync_view context
* selectively enable GL_CULL_FACE after drawing splatting and wireframe (`#96 <https://github.com/koide3/iridescence/issues/96>`_)
* fix wire_frustum and add solid frustum (`#94 <https://github.com/koide3/iridescence/issues/94>`_)
  Co-authored-by: k.koide <k.koide@aist.go.jp>
* guizmo and hovered on subviewers (`#93 <https://github.com/koide3/iridescence/issues/93>`_)
  Co-authored-by: k.koide <k.koide@aist.go.jp>
* metric space point size (`#89 <https://github.com/koide3/iridescence/issues/89>`_)
  * metric space point size
  * update doc
  * update doc
  * point_size instead of point_radius
  ---------
  Co-authored-by: k.koide <k.koide@aist.go.jp>
* partial point attribute update (`#90 <https://github.com/koide3/iridescence/issues/90>`_)
  * partial point attribute update
  * size check
  ---------
  Co-authored-by: k.koide <k.koide@aist.go.jp>
* sub viewer control methods (`#88 <https://github.com/koide3/iridescence/issues/88>`_)
  * sub viewer control methods
  * show sub viewers from the menu
  ---------
  Co-authored-by: k.koide <k.koide@aist.go.jp>
* fov range and regulate pitch angle (`#86 <https://github.com/koide3/iridescence/issues/86>`_)
  Co-authored-by: k.koide <k.koide@aist.go.jp>
* FPSCameraControl (`#77 <https://github.com/koide3/iridescence/issues/77>`_)
  * FPSCameraControl
  * fps camera docs
  * return camera control instance
  * fix python bindings
  * make camera moving speed independent of refresh rate
  ---------
  Co-authored-by: k.koide <k.koide@aist.go.jp>
* plot template methods (`#81 <https://github.com/koide3/iridescence/issues/81>`_)
  Co-authored-by: k.koide <k.koide@aist.go.jp>
* Plot style (`#80 <https://github.com/koide3/iridescence/issues/80>`_)
  * plot_histogram
  * plot_style
  ---------
  Co-authored-by: k.koide <k.koide@aist.go.jp>
* prevent tranparency on ImGui::Image (`#79 <https://github.com/koide3/iridescence/issues/79>`_)
  Co-authored-by: k.koide <k.koide@aist.go.jp>
* fix imgui.input_text (`#78 <https://github.com/koide3/iridescence/issues/78>`_)
  Co-authored-by: k.koide <k.koide@aist.go.jp>
* Keyboard (`#74 <https://github.com/koide3/iridescence/issues/74>`_)
  * better keyboard camera control
  * docs for keyboard shortcut
  * python bindings
  * update docs
  ---------
  Co-authored-by: k.koide <k.koide@aist.go.jp>
* fix plot axes (`#72 <https://github.com/koide3/iridescence/issues/72>`_)
  * fix plot axes
  * plot fitting hotkeey
  * async_wait_until_click and async_toggle_wait
  ---------
  Co-authored-by: k.koide <k.koide@aist.go.jp>
* install implot_internal.h
* doc for spdlog
* async interface (`#71 <https://github.com/koide3/iridescence/issues/71>`_)
  * async interaface
  * fix python build errors
  * python bindings (WIP)
  * plot
  * ordered plot
  * ordered images
  * async interfaces for update\_(image|plot)
  * fix update_image for python
  * fix async_wait()
  * docs
  * plot flags for async_viewer
  ---------
  Co-authored-by: k.koide <k.koide@aist.go.jp>
* Add camera parameter control methods (`#70 <https://github.com/koide3/iridescence/issues/70>`_)
  * accessors for BasicProjectionControl
  * drawable container
  * skip setting model matrix
  * helper functions for static camera control
  ---------
  Co-authored-by: k.koide <k.koide@aist.go.jp>
* title configuration (`#69 <https://github.com/koide3/iridescence/issues/69>`_)
  * title configuration
  * fix example
  ---------
  Co-authored-by: k.koide <k.koide@aist.go.jp>
* Add window flag setting to subviewer (`#67 <https://github.com/koide3/iridescence/issues/67>`_)
  * Add window flag setting to sub viewer
  * Added set_pos binding
* Capture (`#62 <https://github.com/koide3/iridescence/issues/62>`_)
  * screen capture
  * save as jpg
  ---------
  Co-authored-by: k.koide <k.koide@aist.go.jp>
* get_drawables (`#65 <https://github.com/koide3/iridescence/issues/65>`_)
  Co-authored-by: k.koide <k.koide@aist.go.jp>
* add make update_thin_lines return an instance of ThinLines (`#60 <https://github.com/koide3/iridescence/issues/60>`_)
  Co-authored-by: k.koide <k.koide@aist.go.jp>
* suppress ply warning
* add const
* py interface for docking (`#49 <https://github.com/koide3/iridescence/issues/49>`_)
  * py interface for docking
  * more draw_list command for python interface
  * dock builder functions for python
  * image interfaces
  * add arguments for stubgen
  * better texture support
  ---------
  Co-authored-by: k.koide <k.koide@aist.go.jp>
* partial rendering example (`#56 <https://github.com/koide3/iridescence/issues/56>`_)
  Co-authored-by: k.koide <k.koide@aist.go.jp>
* Fix gl test field example (`#57 <https://github.com/koide3/iridescence/issues/57>`_)
  * correctly initialize color buffers, fix gl_test_field_example.cpp
  * correctly load bunny primitive in gl_test_field_example
  * fix typo
  ---------
  Co-authored-by: Julian Gaal <julian.gaal@anavs.de>
* Added zenity install (`#53 <https://github.com/koide3/iridescence/issues/53>`_)
* Ref (`#50 <https://github.com/koide3/iridescence/issues/50>`_)
  * ref
  * const ref
  ---------
  Co-authored-by: k.koide <k.koide@aist.go.jp>
* fix docker permissions: mount `/dev` (`#51 <https://github.com/koide3/iridescence/issues/51>`_)
  * fix docker permissions: mount /dev
  * formatting
  ---------
  Co-authored-by: Julian Gaal <julian.gaal@anavs.de>
* const ref
* const ref
* Find file (`#48 <https://github.com/koide3/iridescence/issues/48>`_)
  * expose find_file
  * indicate source lines causing shader errors
  * inject defines for shaders
  ---------
  Co-authored-by: k.koide <k.koide@aist.go.jp>
* ref (`#47 <https://github.com/koide3/iridescence/issues/47>`_)
  Co-authored-by: k.koide <k.koide@aist.go.jp>
* MapBuffer-based data transfer (`#46 <https://github.com/koide3/iridescence/issues/46>`_)
  * MapBuffer-based data transfer
  * add shorthand methods
  ---------
  Co-authored-by: k.koide <k.koide@aist.go.jp>
* update drawable utility methods (`#45 <https://github.com/koide3/iridescence/issues/45>`_)
  * update drawable utility methods
  * point picking utility
  * viewer instance creation utility
  * fix missing returns
  * remove is_same_v for cuda
  * CMAKE_CXX_STANDARD
  * fix update_points template
  * normal distributions utility
  * efficient normal distributions generation using compute shader
  * fix slow points convertion
  ---------
  Co-authored-by: k.koide <k.koide@aist.go.jp>
* clear_data for SSBO (`#44 <https://github.com/koide3/iridescence/issues/44>`_)
  Co-authored-by: k.koide <k.koide@aist.go.jp>
* shader include (`#42 <https://github.com/koide3/iridescence/issues/42>`_)
  * shader include
  * SSBO copy
  * ssbo copy buffer size option
  * make alpha blending configurable
  * add instant allow/deny filter buttons
  ---------
  Co-authored-by: k.koide <k.koide@aist.go.jp>
* Fix/add imguizmo header (`#43 <https://github.com/koide3/iridescence/issues/43>`_)
  * Added recursive to also pull submodules
  * Added missing boost package installs
  * Changed colromap to colormap
  * Added Imguizmo header
* refactor SSBO (`#41 <https://github.com/koide3/iridescence/issues/41>`_)
  Co-authored-by: k.koide <k.koide@aist.go.jp>
* Added ColorMode RGBA for GridMap (`#40 <https://github.com/koide3/iridescence/issues/40>`_)
  * Added constructor to set RGB independently
  * Added example for gridmap
  * Merged new constructor into original one
  * Rename colortype rgba
  * Fixed iterator
* Update build.yml
* drop bionic build test (`#38 <https://github.com/koide3/iridescence/issues/38>`_)
  * drop bionic build test
  * fix build-args
  ---------
  Co-authored-by: k.koide <k.koide@aist.go.jp>
* fix pattern_formatter include for focal (`#37 <https://github.com/koide3/iridescence/issues/37>`_)
  Co-authored-by: k.koide <k.koide@aist.go.jp>
* Dock (`#36 <https://github.com/koide3/iridescence/issues/36>`_)
  * use imgui docking branch
  * export some imgui headers
  * invoke callbacks before showing special windows
  Co-authored-by: k.koide <k.koide@aist.go.jp>
* Dock (`#35 <https://github.com/koide3/iridescence/issues/35>`_)
  * use imgui docking branch
  * export some imgui headers
  Co-authored-by: k.koide <k.koide@aist.go.jp>
* transparency
* tweak logging window
* use spdlog::ringbuffer_sink
* spdlog_sink
* assimp-based loader (`#33 <https://github.com/koide3/iridescence/issues/33>`_)
  * assimp-based loader
  * 3D model IO
  * resolve relative texture path
  Co-authored-by: k.koide <k.koide@aist.go.jp>
* Create LICENSE
* Update README.md
* Documentation (`#32 <https://github.com/koide3/iridescence/issues/32>`_)
  * writing README (wip)
  * add example images
  * mkdocs (wip)
  * textured mesh
  * mesh and io
  * multi-threading
  * menu bar
  * basic usage
  * cookbook
  * refactor
  Co-authored-by: k.koide <k.koide@aist.go.jp>
* Merge branch 'master' of github.com:koide3/iridescence
* allow updating images in a ui callback
* rename install dependencies script
* fix openpopup python error
* supress warnings
* build test for jammy
* add missing headers for ubuntu 22.04
* march native
* splatting (`#31 <https://github.com/koide3/iridescence/issues/31>`_)
  * splatting
  * remove test code
  * fix wrong texture alpha reference
  Co-authored-by: k.koide <k.koide@aist.go.jp>
* additional constructors for PointCloudBuffer
* bg texture
* install implot.h
* upgrade imgui and introduce implot
* raw pointer inputs for add_intensity
* define CV_16F for old OpenCV
* disable/enable XY grid
* refactor examples
* slight efficiency improvement of ShaderSetting
* Fix UI callbacks (`#30 <https://github.com/koide3/iridescence/issues/30>`_)
  * fix a bug that prevents overriding ui_callbacks
  * viewer window size on python
  Co-authored-by: k.koide <k.koide@aist.go.jp>
* fix a bug that prevents overriding ui_callbacks (`#29 <https://github.com/koide3/iridescence/issues/29>`_)
  Co-authored-by: k.koide <k.koide@aist.go.jp>
* hovered drawings (`#28 <https://github.com/koide3/iridescence/issues/28>`_)
  * show texts in backend
  * hovered drawings
  * hovered example
  Co-authored-by: k.koide <k.koide@aist.go.jp>
* Image group (`#27 <https://github.com/koide3/iridescence/issues/27>`_)
  * image grouping
  * point picking
  Co-authored-by: k.koide <k.koide@aist.go.jp>
* raw pointer input for NormalDistributions
* raw pointer input for glk::Lines
* refactor normal distributions interface and fix improper mesh vao deletion bug (`#26 <https://github.com/koide3/iridescence/issues/26>`_)
  Co-authored-by: k.koide <k.koide@aist.go.jp>
* colored mesh (`#25 <https://github.com/koide3/iridescence/issues/25>`_)
  * colored mesh
  * jpeg io
  * add libjpeg dependency
  * read_depth_pixels
  * refactor examples
  * static camera control
  * pixel buffer object
  * pbo example
  * add partial rendering clear thresh
  Co-authored-by: k.koide <k.koide@aist.go.jp>
* add size accessors
* toggle_spin_once
* add flat colors to python
* add missing PYTHON_BINDINGS flag
* Indexed lines (`#24 <https://github.com/koide3/iridescence/issues/24>`_)
  * fix ply ascii loading
  * indexed point cloud buffer
  * rename drawable filter registration method
  * add progress interfaces
  * support std::allocator
  * make_shared
  Co-authored-by: k.koide <k.koide@aist.go.jp>
* Devel (`#23 <https://github.com/koide3/iridescence/issues/23>`_)
  * add regex remove_drawable
  * writing binary ply loader (WIP)
  * auto ply loading for ascii and binary
  * remove ply test code
  * decimal point cloud rendering
  * remove .vscode
  * remove test code
  * partial point cloud rendering
  Co-authored-by: k.koide <k.koide@aist.go.jp>
* Devel (`#22 <https://github.com/koide3/iridescence/issues/22>`_)
  * add regex remove_drawable
  * writing binary ply loader (WIP)
  * auto ply loading for ascii and binary
  * remove ply test code
  * decimal point cloud rendering
  * remove .vscode
  * remove test code
  Co-authored-by: k.koide <k.koide@aist.go.jp>
* Neo (`#21 <https://github.com/koide3/iridescence/issues/21>`_)
  * eliminate catkin
  * add install targets
  * add default build type
  * remove build test for ROS
  * add (fake) arcball control
  * add vsync menu
  * autoresize
  * cast
  * draw coordinate systems with lines
  * add destroy()
  * file IO
  * install pfd
  * add progress
  * PointCloudBuffer constructor with vector<Vector4f>
  * add_color with Vector4d
  * fix default cmake build type
  * hide imguizmo header
  * is_guizmo_using
  * split text by newlines
  * fix cmake errors on bionic
  Co-authored-by: k.koide <k.koide@aist.go.jp>
* Neo (`#20 <https://github.com/koide3/iridescence/issues/20>`_)
  * eliminate catkin
  * add install targets
  * add default build type
  * remove build test for ROS
  * add (fake) arcball control
  * add vsync menu
  * autoresize
  * cast
  * draw coordinate systems with lines
  * add destroy()
  * file IO
  * install pfd
  * add progress
  * PointCloudBuffer constructor with vector<Vector4f>
  * add_color with Vector4d
  * fix default cmake build type
  * hide imguizmo header
  * is_guizmo_using
  Co-authored-by: k.koide <k.koide@aist.go.jp>
* add PointCloudBuffer utility constructors (`#19 <https://github.com/koide3/iridescence/issues/19>`_)
  Co-authored-by: k.koide <k.koide@aist.go.jp>
* Image (`#18 <https://github.com/koide3/iridescence/issues/18>`_)
  * add frustum and image interface
  * add frustum.hpp
  * add texture_opencv.hpp
  * fix build error on melodic
  Co-authored-by: k.koide <k.koide@aist.go.jp>
* Merge pull request `#17 <https://github.com/koide3/iridescence/issues/17>`_ from koide3/devel
  add ssli config to LightViewer UI
* Update README.md
* add ssli config to LightViewer UI
* Merge pull request `#16 <https://github.com/koide3/iridescence/issues/16>`_ from koide3/devel
  add wire primitives
* add wire primitives
* Merge pull request `#15 <https://github.com/koide3/iridescence/issues/15>`_ from koide3/devel
  Devel
* guizmo on subviewer
* LightViewerContext methods
* add CameraControl to pybind
* Merge branch 'devel' of github.com:koide3/iridescence into devel
* add PointCloudBuffer python methods
* Merge pull request `#14 <https://github.com/koide3/iridescence/issues/14>`_ from koide3/splat
  add SSLI_SPLAT
* Merge pull request `#13 <https://github.com/koide3/iridescence/issues/13>`_ from koide3/camera
  add camera save/load
* add SSLI_SPLAT
* add camera save/load
* Merge pull request `#12 <https://github.com/koide3/iridescence/issues/12>`_ from koide3/koide3-patch-1
  Migrate to Github Actions
* Merge branch 'koide3-patch-1' of github.com:koide3/iridescence into koide3-patch-1
* bye travis
* add ca-certificates
* Update build.yml
* Create build.yml
* Merge pull request `#11 <https://github.com/koide3/iridescence/issues/11>`_ from koide3/util
  Util
* add .vscode to .gitignore
* add colormaps
* add enable_vsync
* add colored points
* Merge pull request `#10 <https://github.com/koide3/iridescence/issues/10>`_ from koide3/util
  add directional lighting
* add fancy buttons
* Merge branch 'master' of github.com:koide3/iridescence into util
* Merge branch 'util' of github.com:koide3/iridescence into util
* reset windowShouldClose flag when calling spin
* revert docker login
* bye docker caching
* add AtomicCounters & ShaderStorageBuffer
* splatting working
* eigensolver working
* going to use 2D radius for finalized buffer
* improve knn speed (still not satisfying)
* knn working
* profiling
* knn working but slowly (WIP)
* point cloud splatting (WIP~
* add directional lighting
* Merge pull request `#9 <https://github.com/koide3/iridescence/issues/9>`_ from koide3/util
  add utility constructors
* add utility constructors
* Merge pull request `#8 <https://github.com/koide3/iridescence/issues/8>`_ from koide3/pybind
  add lines to pyridescence
* refactor dockerfiles
* refactor Dockerfiles
* reduce docker pull
* add lines to pyridescence
* Merge pull request `#7 <https://github.com/koide3/iridescence/issues/7>`_ from koide3/pybind
  Pybind
* info and depth picking (info picking seems have some bug)
* imgui inputs
* a bit more imgui functions
* build without ninja
* add python example
* separate definitions
* install data files in python
* python bindings
* Merge pull request `#5 <https://github.com/koide3/iridescence/issues/5>`_ from koide3/devel
  Devel
* add screenshot saver
* refactoring and debug effect screen bug
* add anonymous namer
* better support for sub_viewer
* light attenuation
* Merge branch 'devel' of github.com:koide3/iridescence into devel
* support texture color and normal buffer
* keyboard input
* Merge pull request `#4 <https://github.com/koide3/iridescence/issues/4>`_ from koide3/devel
  Devel
* maximize window
* colored thin lines
* use material's alpha
* transparency
* change screen effect rendering scheme
* be quiet
* avoid static const member initialization
* refactoring
* handle hidden window
* add trajectory
* Merge pull request `#3 <https://github.com/koide3/iridescence/issues/3>`_ from koide3/brdf
  update iridescence shader
* update iridescence shader
* Merge pull request `#2 <https://github.com/koide3/iridescence/issues/2>`_ from koide3/brdf
  Brdf
* add libpng
* add iridescence brdf
* add several BRDF models
* Merge pull request `#1 <https://github.com/koide3/iridescence/issues/1>`_ from koide3/devel
  add colormaps
* change package name (again)
* add NormalDistributions
* add imguizmo include
* add camera utilities
* add camera contral utility
* get rid of g2o
* accept Eigen::Transform for ShaderSetting
* add graph drawable
* factor graph (preliminary)
* information picking
* change SSAO num samples
* multi-light
* iridescent lighting
* screen space lighting
* colormap
* add colormaps
* viewer UI
* change package name
* add intensities
* register ROS docker
* add ROS docker
* add bionic docker
* travis
* define built_in_ros
* check if it's built in catkin_ws
* unified cmake/catkin build
* refactor CMakeLists.txt
* fix duplicated definition
* refactoring
* add colored points
* update light_viewer_basic
* picking
* add multiple file push
* add recent_files
* add install.sh
* fix version related bug
* multi-thread example
* add drawable filter
* delete pcl point cloud buffer constructor
* add invoke
* data path management
* add primitive wireframes
* merge
* add global shader setting
* sub canvas for light_viewer
* adaptive depth range
* Merge branch 'catkin' of mobarobo.tk:KenjiKoide/gl_test_field into catkin
* GL interop
* add screen effect
* add light_viewer_kitti
* add light_viewer_custom
* add light_viewer_basic
* add colormap
* add some examples
* add imguizmo and make pcl optional
* add find_drawable
* shiftable lookat
* remove num_points display
* fix
* add cpu info
* add selective clear drawables
* add clear_drawables()
* Merge branch 'catkin' of https://bitbucket.org/koide3/gl_test_field into catkin
* add raw pointer construction for PointCloudBuffer
* fix dependency
* templatenize PointCloudBuffer constructor
* add framebuffer size getter
* add utilities
* update viewer
* save
* add bunny to primitive list
* refactoring
* initial commit
* Contributors: Julian Gaal, Martin Valgur, Patiphon Narksri, Seekerzero, Taro Tako, Tomoya Sato, k.koide, kenji koide, koide3
