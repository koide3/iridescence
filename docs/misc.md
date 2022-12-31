# Miscellaneous


## Text output

```cpp
viewer->append_text("text1");
viewer->append_text("text2");
```

## Remove drawables

```cpp
// Remove a drawable with a specified name
viewer->remove_drawable("drawable_name");

// Removes drawables with names that match a regex pattern
viewer->remove_drawable(std::regex("drawable_.+"));

// Remove all drawables
viewer->clear_drawables();
```


## Drawable filter

```cpp
viewer->register_drawable_filter("filter", [](const std::string& drawable_name) {
  bool do_rendering = true;

  if (drawable_name == "drawable_to_be_filtered") {
    do_rendering = false;
  }

  return do_rendering;
});

// Drawable filter can be cleared by giving 0
viewer->register_drawable_filter("filter", 0);
```

## Sub-viewer

```cpp
auto sub_viewer1 = viewer->sub_viewer("sub1");
sub_viewer1->update_drawable("cube", glk::Primitives::cube(), guik::Rainbow());

auto sub_viewer2 = viewer->sub_viewer("sub2");
sub_viewer2->update_drawable("sphere", glk::Primitives::sphere(), guik::Rainbow());
```

## Share the default camera control with sub-viewers

```cpp
auto camera_control = viewer->get_camera_control();
sub_viewer1->set_camera_control(camera_control);
sub_viewer2->set_camera_control(camera_control);
```

## File dialogs (portable-file-dialogs)

```cpp
#include <portable-file-dialogs.h>
#include <guik/recent_files.hpp>

guik::RecentFiles recent_files("input_directory");
const std::string path = pfd::select_folder("Select input directory", recent_files.most_recent()).result();
if (!path.empty()) {
  recent_files.push(path);
}
```

