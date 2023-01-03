# Screen space effect

(Documentation unfinished)

## Screen space attribute estimation

![ss](https://user-images.githubusercontent.com/31344317/210160116-b1456193-a12d-4681-acbf-49964ec3faa9.png)

## Input image without screen effects

[![Screenshot_20230101_160408](https://user-images.githubusercontent.com/31344317/210163351-acb4540c-91b7-4cff-94ea-668afce822a2.png){: style="height:300px;width:300px"}](https://user-images.githubusercontent.com/31344317/210163351-acb4540c-91b7-4cff-94ea-668afce822a2.png)

## Naive screen space ambient occlusion

```cpp
#include <glk/effects/naive_screen_space_ambient_occlusion.hpp>

auto ssao = std::make_shared<glk::NaiveScreenSpaceAmbientOcclusion>();
viewer->set_screen_effect(ssao);
```

[![ss_1672541947 275343](https://user-images.githubusercontent.com/31344317/210159856-039a9a61-bd1c-4df9-8d14-836aba2d8ee6.png){: style="height:300px;width:300px"}](https://user-images.githubusercontent.com/31344317/210159856-039a9a61-bd1c-4df9-8d14-836aba2d8ee6.png)

## Smoothed screen space ambient occlusion

```cpp
#include <glk/effects/screen_space_ambient_occlusion.hpp>

auto ssao = std::make_shared<glk::ScreenSpaceAmbientOcclusion>();
viewer->set_screen_effect(ssao);
```

[![ss_1672541948 458399](https://user-images.githubusercontent.com/31344317/210159857-720a962a-238b-4026-9cab-f69de13ee8d2.png){: style="height:300px;width:300px"}](https://user-images.githubusercontent.com/31344317/210159857-720a962a-238b-4026-9cab-f69de13ee8d2.png)


## Screen space lighting

```cpp
#include <glk/effects/screen_space_lighting.hpp>

Eigen::Vector3f light0_pos(1.0f, 1.0f, 5.0f);
Eigen::Vector4f light0_color(2.0f, 2.0f, 2.0f, 1.0f);

Eigen::Vector3f light1_pos(0.0f, -2.0f, 2.0f);
Eigen::Vector4f light1_color(0.5f, 0.5f, 0.5f, 1.0f);

auto lighting_effect = std::make_shared<glk::ScreenSpaceLighting>();
lighting_effect->set_light(0, light0_pos, light0_color);
lighting_effect->set_light(1, light1_pos, light1_color);

viewer->enable_normal_buffer();
viewer->set_screen_effect(lighting_effect);
```

[![ss_1672541955 615492](https://user-images.githubusercontent.com/31344317/210159915-52bf0439-704d-43cc-aacf-55711d503a28.png){: style="height:300px;width:300px"}](https://user-images.githubusercontent.com/31344317/210159915-52bf0439-704d-43cc-aacf-55711d503a28.png)
[![ss_1672541966 604093](https://user-images.githubusercontent.com/31344317/210159858-61e38dab-7954-4ff6-986f-7ef9469fed15.png){: style="height:300px;width:300px"}](https://user-images.githubusercontent.com/31344317/210159858-61e38dab-7954-4ff6-986f-7ef9469fed15.png)
