#!/bin/env python
import numpy
from pyridescence import *

class TestUI:
  def __init__(self):
    self.viewer = guik.viewer()
    self.viewer.register_ui_callback('test_ui', self.ui_callback)
  
  def spin(self):
    self.viewer.spin()
  
  def io_test(self):
    io = imgui.get_io()
    imgui.text(f'wants_capture_mouse: {io.want_capture_mouse}')
    imgui.text(f'wants_capture_keyboard: {io.want_capture_keyboard}')
    imgui.text(f'framerate: {io.framerate}')
    imgui.text(f'delta_time: {io.delta_time}')
    imgui.text(f'mouse_pos: {io.mouse_pos[0]}, {io.mouse_pos[1]}')
    imgui.text(f'mouse_down: {io.mouse_down[0]}, {io.mouse_down[1]}, {io.mouse_down[2]}')
    imgui.text(f'mouse_clicked: {io.mouse_clicked[0]}, {io.mouse_clicked[1]}, {io.mouse_clicked[2]}')
    imgui.text(f'mouse_wheel: {io.mouse_wheel}')
    imgui.text(f'mouse_wheel_h: {io.mouse_wheel_h}')
    imgui.text(f'key_ctrl: {str(io.key_ctrl)}')
    imgui.text(f'key_shift: {str(io.key_shift)}')
    imgui.text(f'key_alt: {str(io.key_alt)}')
    imgui.text(f'key_super: {str(io.key_super)}')

    pressed_keys = [imgui.get_key_name(i) for i in range(len(io.keys_down)) if io.keys_down[i]]
    imgui.text(f'pressed_keys: {", ".join(pressed_keys)}')
    imgui.text(f'nav_inputs:{", ".join([f"{i}:{io.nav_inputs[i]:.2f}" for i in range(len(io.nav_inputs))])}')

  def style_test(self):
    style = imgui.get_style()
    
    imgui.text(f'alpha: {style.alpha:.2f}')
    imgui.text(f'disabled_alpha: {style.disabled_alpha:.2f}')
    imgui.text(f'window_padding: ({style.window_padding.x:.2f}, {style.window_padding.y:.2f})')
    imgui.text(f'window_rounding: {style.window_rounding:.2f}')
    imgui.text(f'window_border_size: {style.window_border_size:.2f}')
    imgui.text(f'window_min_size: ({style.window_min_size.x:.2f}, {style.window_min_size.y:.2f})')
    imgui.text(f'window_title_align: ({style.window_title_align.x:.2f}, {style.window_title_align.y:.2f})')
    imgui.text(f'window_menu_button_position: {style.window_menu_button_position}')
    imgui.text(f'child_rounding: {style.child_rounding:.2f}')
    imgui.text(f'child_border_size: {style.child_border_size:.2f}')
    imgui.text(f'popup_rounding: {style.popup_rounding:.2f}')
    imgui.text(f'popup_border_size: {style.popup_border_size:.2f}')
    imgui.text(f'frame_padding: ({style.frame_padding.x:.2f}, {style.frame_padding.y:.2f})')
    imgui.text(f'frame_rounding: {style.frame_rounding:.2f}')
    imgui.text(f'frame_border_size: {style.frame_border_size:.2f}')
    imgui.text(f'item_spacing: ({style.item_spacing.x:.2f}, {style.item_spacing.y:.2f})')
    imgui.text(f'item_inner_spacing: ({style.item_inner_spacing.x:.2f}, {style.item_inner_spacing.y:.2f})')
    imgui.text(f'cell_padding: ({style.cell_padding.x:.2f}, {style.cell_padding.y:.2f})')
    imgui.text(f'touch_extra_padding: ({style.touch_extra_padding.x:.2f}, {style.touch_extra_padding.y:.2f})')
    imgui.text(f'indent_spacing: {style.indent_spacing:.2f}')
    imgui.text(f'columns_min_spacing: {style.columns_min_spacing:.2f}')
    imgui.text(f'scrollbar_size: {style.scrollbar_size:.2f}')
    imgui.text(f'scrollbar_rounding: {style.scrollbar_rounding:.2f}')
    imgui.text(f'grab_min_size: {style.grab_min_size:.2f}')
    imgui.text(f'grab_rounding: {style.grab_rounding:.2f}')
    imgui.text(f'tab_rounding: {style.tab_rounding:.2f}')
    imgui.text(f'tab_border_size: {style.tab_border_size:.2f}')
    imgui.text(f'tab_min_width_for_close_button: {style.tab_min_width_for_close_button:.2f}')
    imgui.text(f'color_button_position: {style.color_button_position}')
    imgui.text(f'button_text_align: ({style.button_text_align.x:.2f}, {style.button_text_align.y:.2f})')
    imgui.text(f'selectable_text_align: ({style.selectable_text_align.x:.2f}, {style.selectable_text_align.y:.2f})')
    imgui.text(f'display_window_padding: ({style.display_window_padding.x:.2f}, {style.display_window_padding.y:.2f})')
    imgui.text(f'display_safe_area_padding: ({style.display_safe_area_padding.x:.2f}, {style.display_safe_area_padding.y:.2f})')
    imgui.text(f'mouse_cursor_scale: {style.mouse_cursor_scale:.2f}')
    imgui.text(f'anti_aliased_lines: {style.anti_aliased_lines}')
    imgui.text(f'anti_aliased_lines_use_tex: {style.anti_aliased_lines_use_tex}')
    imgui.text(f'anti_aliased_fill: {style.anti_aliased_fill}')
    imgui.text(f'curve_tessellation_tol: {style.curve_tessellation_tol:.2f}')
    imgui.text(f'circle_tessellation_max_error: {style.circle_tessellation_max_error:.2f}')
    
    imgui.separator()
    imgui.text('=== Colors (first 10) ===')
    colors = style.colors
    for i in range(min(10, len(colors))):
      color = colors[i]
      imgui.text(f'Color[{i}]: ({color[0]:.2f}, {color[1]:.2f}, {color[2]:.2f}, {color[3]:.2f})')

  def drawlist_test(self):
    imgui.text('DrawList Test:')
    if not hasattr(self, 'dl_mode'):
      self.dl_mode = 0

    if imgui.radio_button('fg', self.dl_mode == 0):
      self.dl_mode = 0
    imgui.same_line()
    if imgui.radio_button('bg', self.dl_mode == 1):
      self.dl_mode = 1
    imgui.same_line()
    if imgui.radio_button('window', self.dl_mode == 2):
      self.dl_mode = 2

    if self.dl_mode == 0:
      dl = imgui.get_foreground_draw_list()
    elif self.dl_mode == 1:
      dl = imgui.get_background_draw_list()
    else:
      dl = imgui.get_window_draw_list()

    dl.add_line((100, 100), (200, 100), imgui.IM_COL32(255, 0, 0, 255), 3.0)
    dl.add_line((300, 100), (400, 100), imgui.IM_COL32(0, 255, 0, 128), 2.0)
    dl.add_line((500, 100), (600, 100), imgui.IM_COL32(0, 0, 255, 64), 1.0)
    
    dl.add_rect_filled((100, 200), (200, 250), imgui.IM_COL32(255, 0, 0, 255), rounding=0.0)
    dl.add_rect_filled((300, 200), (400, 250), imgui.IM_COL32(0, 255, 0, 128), rounding=10.0)
    dl.add_rect_filled((500, 200), (600, 250), imgui.IM_COL32(0, 0, 255, 64), rounding=20.0)

    dl.add_rect((100, 300), (200, 350), imgui.IM_COL32(255, 0, 0, 255), thickness=3.0, rounding=0.0)
    dl.add_rect((300, 300), (400, 350), imgui.IM_COL32(0, 255, 0, 128), thickness=2.0, rounding=10.0)
    dl.add_rect((500, 300), (600, 350), imgui.IM_COL32(0, 0, 255, 64), thickness=1.0, rounding=20.0)
    
    dl.add_circle_filled((150, 450), 25.0, imgui.IM_COL32(255, 0, 0, 255), num_segments=0)
    dl.add_circle_filled((350, 450), 25.0, imgui.IM_COL32(0, 255, 0, 128), num_segments=0)
    dl.add_circle_filled((550, 450), 25.0, imgui.IM_COL32(0, 0, 255, 64), num_segments=0)

    dl.add_circle((150, 550), 25.0, imgui.IM_COL32(255, 0, 0, 255), num_segments=0, thickness=3.0)
    dl.add_circle((350, 550), 25.0, imgui.IM_COL32(0, 255, 0, 128), num_segments=0, thickness=2.0)
    dl.add_circle((550, 550), 25.0, imgui.IM_COL32(0, 0, 255, 64), num_segments=0, thickness=1.0)

    dl.add_text((100, 650), imgui.IM_COL32(255, 255, 255, 255), 'This is a test text drawn by ImDrawList.')

  def demo_test(self):
    if not hasattr(self, 'show_demo'):
      self.show_demo = False
      self.show_metrics = False
      self.show_debug_log = False
      self.show_style_editor = False
    
    imgui.text(f'version: {imgui.get_version()}')

    imgui.text('style colors:')
    if imgui.button('dark'):
      imgui.style_colors_dark()
    imgui.same_line()
    if imgui.button('light'):
      imgui.style_colors_light()
    imgui.same_line()
    if imgui.button('classic'):
      imgui.style_colors_classic()
  
    imgui.show_style_selector('styles')
    imgui.show_font_selector('fonts')
    
    _, self.show_demo = imgui.checkbox('Show ImGui Demo Window', self.show_demo)
    if self.show_demo:
      imgui.show_demo_window()
    
    _, self.show_metrics = imgui.checkbox('Show ImGui Metrics Window', self.show_metrics)
    if self.show_metrics:
      imgui.show_metrics_window()
    
    _, self.show_debug_log = imgui.checkbox('Show ImGui Debug Log Window', self.show_debug_log)
    if self.show_debug_log:
      imgui.show_debug_log_window()
    
    _, self.show_style_editor = imgui.checkbox('Show ImGui Style Editor', self.show_style_editor)
    if self.show_style_editor:
      imgui.show_style_editor()

  def win_test(self):
    if imgui.begin_child('child', (300, 200), border=True):
      imgui.text('in child window')
      imgui.end_child()

    imgui.text(f'is_window_appearing: {imgui.is_window_appearing()}')
    imgui.text(f'is_window_collapsed: {imgui.is_window_collapsed()}')
    imgui.text(f'is_window_focused: {imgui.is_window_focused()}')
    imgui.text(f'is_window_hovered: {imgui.is_window_hovered()}')
    imgui.text(f'get_window_pos: ({imgui.get_window_pos()[0]}, {imgui.get_window_pos()[1]})')
    imgui.text(f'get_window_size: ({imgui.get_window_size()[0]}, {imgui.get_window_size()[1]})')
    imgui.text(f'get_window_width: {imgui.get_window_width()}')
    imgui.text(f'get_window_height: {imgui.get_window_height()}')

    imgui.text(f'get_content_region_avail: ({imgui.get_content_region_avail()[0]}, {imgui.get_content_region_avail()[1]})')
    imgui.text(f'get_content_region_max: ({imgui.get_content_region_max()[0]}, {imgui.get_content_region_max()[1]})')
    imgui.text(f'get_window_content_region_min: ({imgui.get_window_content_region_min()[0]}, {imgui.get_window_content_region_min()[1]})')
    imgui.text(f'get_window_content_region_max: ({imgui.get_window_content_region_max()[0]}, {imgui.get_window_content_region_max()[1]})')

  def param_stacks_test(self):
    if not hasattr(self, 'repeat_value'):
      self.repeat_value = 0

    imgui.push_button_repeat(True)
    if imgui.button('Repeat Button'):
      self.repeat_value += 1
    imgui.same_line()
    imgui.text(f'value: {self.repeat_value}')
    imgui.pop_button_repeat()

    imgui.push_item_width(150)
    imgui.drag_float('fl (w=150)', 0.5)
    imgui.pop_item_width()
    imgui.text(f'calc_item_width: {imgui.calc_item_width()}')

    imgui.set_next_item_width(250)
    imgui.drag_float('fl (w=250)', 0.5)
    imgui.text(f'calc_item_width: {imgui.calc_item_width()}')

    imgui.push_text_wrap_pos(200)
    imgui.text('This is a long text that is wrapped at 200 pixels width. ' * 3)
    imgui.pop_text_wrap_pos()
  
  def cursor_test(self):
    imgui.separator()

    imgui.text('Text')
    imgui.same_line()
    imgui.text('Text after same_line')

    imgui.newline()
    imgui.text('Text after newline')

    imgui.spacing()
    imgui.text('Text after spacing')

    imgui.dummy((0, 20))
    imgui.text('Text after dummy(0,20)')

    imgui.indent(20)
    imgui.text('Text after indent(20)')
    imgui.text('More text after indent(20)')

    imgui.unindent(20)
    imgui.text('Text after unindent(20)')

    imgui.text(f'get_cursor_pos: ({imgui.get_cursor_pos()[0]}, {imgui.get_cursor_pos()[1]})')
    imgui.text(f'get_cursor_pos_x: {imgui.get_cursor_pos_x()}')
    imgui.text(f'get_cursor_pos_y: {imgui.get_cursor_pos_y()}')
    imgui.text(f'get_cursor_screen_pos: ({imgui.get_cursor_screen_pos()[0]}, {imgui.get_cursor_screen_pos()[1]})')
    imgui.text(f'get_text_line_height: {imgui.get_text_line_height()}')
    imgui.text(f'get_text_line_height_with_spacing: {imgui.get_text_line_height_with_spacing()}')

  def text_test(self):
    imgui.text_unformatted('text_unformatted')
    imgui.text('text')
    imgui.text_colored((1.0, 0.0, 0.0, 1.0), 'text_colored (red)')
    imgui.text_disabled('text_disabled')
    imgui.label_text('label1', 'label_text1')
    imgui.label_text('label2', 'label_text2')
    imgui.bullet_text('bullet_text 1')
    imgui.bullet_text('bullet_text 2')
  
  def main_widget_test(self):
    if not hasattr(self, 'main_checked'):
      self.main_checked = False
      self.main_flags = 0
      self.main_radio = 0
      self.main_radio2 = 0

    if imgui.button('press me'):
      imgui.same_line()
      imgui.text('pressed!!')
    
    imgui.text('invisible button test:')
    if imgui.invisible_button('invisible_btn', (100, 50)):
      imgui.same_line()
      imgui.text('invisible button pressed!!')
    
    if imgui.arrow_button('arrow_btn', imgui.Dir_Left):
      imgui.same_line()
      imgui.text('arrow button pressed!!')
    
    _, self.main_checked = imgui.checkbox('checkbox', self.main_checked)
    _, self.main_flags = imgui.checkbox_flags('flag1', self.main_flags, 1)
    imgui.same_line()
    _, self.main_flags = imgui.checkbox_flags('flag2', self.main_flags, 2)
    imgui.same_line()
    _, self.main_flags = imgui.checkbox_flags('flag3', self.main_flags, 4)

    if imgui.radio_button('radio1', self.main_radio == 0):
      self.main_radio = 0
    if imgui.radio_button('radio2', self.main_radio == 1):
      self.main_radio = 1
    if imgui.radio_button('radio3', self.main_radio == 2):
      self.main_radio = 2

    _, self.main_radio2 = imgui.radio_button_int('radio_int1', self.main_radio2, 0)
    _, self.main_radio2 = imgui.radio_button_int('radio_int2', self.main_radio2, 1)
    _, self.main_radio2 = imgui.radio_button_int('radio_int3', self.main_radio2, 2)

    imgui.progress_bar((imgui.get_time() * 0.5) % 1.0, (200, 20), 'progress_bar')
    imgui.bullet()
    imgui.same_line()
    imgui.text('bullet text in main widget')
  
  def image_test(self):
    if not hasattr(self, 'image_texture'):
      colors = numpy.array([glk.colormapf(glk.COLORMAP.TURBO, x / 128.0) for x in range(128)]).reshape(1, 128, 4)
      img = colors.repeat(128, axis=0).astype(numpy.float32)
      self.image_texture = glk.create_texture_f(img)
    
    imgui.image(self.image_texture, (256, 256))

    if imgui.image_button('image_button', self.image_texture, (64, 64)):
      imgui.same_line()
      imgui.text('image button pressed!!')

  def combo_test(self):
    if not hasattr(self, 'combo_selected'):
      self.combo_selected = 'option 0'
      self.combo_selected2 = 0

    if imgui.begin_combo('combo', 'select an option'):
      for i in range(5):
        if imgui.selectable(f'option {i}'):
          imgui.same_line()
          self.combo_selected = f'option {i}'
      imgui.end_combo()

    imgui.text(f'selected: {self.combo_selected}')

    _, self.combo_selected2 = imgui.combo('combo_int', self.combo_selected2, [f'item {i}' for i in range(5)])

  def drag_test(self):
    if not hasattr(self, 'drag_float'):
      self.drag_float = 0.0
      self.drag_float2 = [0.0, 1.0]
      self.drag_float3 = [0.0, 1.0, 2.0]
      self.drag_float4 = [0.0, 1.0, 2.0, 3.0]
    
      self.drag_int = 0
      self.drag_int2 = [0, 1]
      self.drag_int3 = [0, 1, 2]
      self.drag_int4 = [0, 1, 2, 3]

    _, self.drag_float = imgui.drag_float('drag_float1', self.drag_float, 0.1, 0.0, 10.0)
    _, self.drag_float2 = imgui.drag_float2('drag_float2', self.drag_float2, 0.1, 0.0, 10.0)
    _, self.drag_float3 = imgui.drag_float3('drag_float3', self.drag_float3, 0.1, 0.0, 10.0)
    _, self.drag_float4 = imgui.drag_float4('drag_float4', self.drag_float4, 0.1, 0.0, 10.0)
    _, self.drag_float2 = imgui.drag_float_range2('drag_float_range2', self.drag_float2, 0.1, 0.0, 10.0)

    _, self.drag_int = imgui.drag_int('drag_int1', self.drag_int, 1, 0, 100)
    _, self.drag_int2 = imgui.drag_int2('drag_int2', self.drag_int2, 1, 0, 100)
    _, self.drag_int3 = imgui.drag_int3('drag_int3', self.drag_int3, 1, 0, 100)
    _, self.drag_int4 = imgui.drag_int4('drag_int4', self.drag_int4, 1, 0, 100)
    _, self.drag_int2 = imgui.drag_int_range2('drag_int_range2', self.drag_int2, 1, 0, 100)

    _, self.drag_float = imgui.slider_float('slider_float', self.drag_float, 0.0, 10.0)
    _, self.drag_float2 = imgui.slider_float2('slider_float2', self.drag_float2, 0.0, 10.0)
    _, self.drag_float3 = imgui.slider_float3('slider_float3', self.drag_float3, 0.0, 10.0)
    _, self.drag_float4 = imgui.slider_float4('slider_float4', self.drag_float4, 0.0, 10.0)
    _, self.drag_float = imgui.slider_angle('slider_angle', self.drag_float)

    _, self.drag_int = imgui.slider_int('slider_int', self.drag_int, 0, 100)
    _, self.drag_int2 = imgui.slider_int2('slider_int2', self.drag_int2, 0, 100)
    _, self.drag_int3 = imgui.slider_int3('slider_int3', self.drag_int3, 0, 100)
    _, self.drag_int4 = imgui.slider_int4('slider_int4', self.drag_int4, 0, 100)

  def input_text_test(self):
    if not hasattr(self, 'input_text'):
      self.input_text = 'input text'
      self.input_text_multiline = 'This is a multiline\ninput text box.\nYou can write multiple lines here.'
      self.input_text_with_hint = ''
      self.input_float = 0.0
      self.input_float2 = [0.0, 1.0]
      self.input_float3 = [0.0, 1.0, 2.0]
      self.input_float4 = [0.0, 1.0, 2.0, 3.0]
      self.input_int = 0
      self.input_int2 = [0, 1]
      self.input_int3 = [0, 1, 2]
      self.input_int4 = [0, 1, 2, 3]

    _, self.input_text = imgui.input_text('input_text', self.input_text)
    _, self.input_text_multiline = imgui.input_text_multiline('input_text_multiline', self.input_text_multiline, (300, 100))
    _, self.input_text_with_hint = imgui.input_text_with_hint('input_text_with_hint', 'enter text here...', self.input_text_with_hint)
    _, self.input_float = imgui.input_float('input_float', self.input_float, 0.1, 1.0)
    _, self.input_float2 = imgui.input_float2('input_float2', self.input_float2)
    _, self.input_float3 = imgui.input_float3('input_float3', self.input_float3)
    _, self.input_float4 = imgui.input_float4('input_float4', self.input_float4)
    _, self.input_int = imgui.input_int('input_int', self.input_int)
    _, self.input_int2 = imgui.input_int2('input_int2', self.input_int2)
    _, self.input_int3 = imgui.input_int3('input_int3', self.input_int3)
    _, self.input_int4 = imgui.input_int4('input_int4', self.input_int4)

  def color_test(self):
    if not hasattr(self, 'color3'):
      self.color3 = [1.0, 0.0, 0.0]
      self.color4 = [0.0, 1.0, 0.0, 1.0]

    _, self.color3 = imgui.color_edit3('color3', self.color3)
    _, self.color4 = imgui.color_edit4('color4', self.color4)
    _, self.color3 = imgui.color_picker3('color_picker3', self.color3)
    _, self.color4 = imgui.color_picker4('color_picker4', self.color4)

    if imgui.color_button('color_button', self.color4):
      imgui.same_line()
      imgui.text('color button pressed!!')
    
  def trees_test(self):
    if imgui.tree_node('tree node 1'):
      imgui.text('inside tree node 1')
      if imgui.tree_node('tree node 2'):
        imgui.text('inside tree node 2')
        imgui.tree_pop()
      imgui.tree_pop()

  def menu_test(self):
    if imgui.begin_main_menu_bar():
      if imgui.begin_menu('menu'):
        if imgui.menu_item('menu_item 1'):
          pass
        if imgui.menu_item('menu_item 2'):
          pass
        if imgui.begin_menu('menux'):
          imgui.menu_item('menu_item x1')

          imgui.end_menu()

        imgui.end_menu()

      if imgui.begin_menu('menu2'):
        if imgui.menu_item('menu_item A'):
          pass
        if imgui.menu_item('menu_item B'):
          pass
        imgui.end_menu()

      imgui.end_main_menu_bar()

  def popup_test(self):
    if not hasattr(self, 'modal_opened'):
      self.modal_opened = False

    imgui.text('tooltip')
    if imgui.is_item_hovered():
      imgui.begin_tooltip()
      imgui.text('this is a tooltip')
      imgui.end_tooltip()
    
    if imgui.button('open popup'):
      imgui.open_popup('popup1')
    
    if imgui.begin_popup('popup1', imgui.WindowFlags_AlwaysAutoResize):
      imgui.text('this is a popup')
      if imgui.button('close'):
        imgui.close_current_popup()
      imgui.end_popup()
    imgui.text(f'popup open: {imgui.is_popup_open("popup1")}')
    
    if imgui.button('open modal'):
      imgui.open_popup('modal1')
    
    if imgui.begin_popup_modal('modal1', imgui.WindowFlags_AlwaysAutoResize):
      imgui.text('this is a modal popup')
      if imgui.button('close'):
        imgui.close_current_popup()
      imgui.end_popup()

  def table_test(self):
    imgui.begin_table('table1', 3, imgui.TableFlags_Borders | imgui.TableFlags_RowBg)
    for row in range(3):
      imgui.table_next_row()
      for column in range(3):
        imgui.table_set_column_index(column)
        imgui.text(f'Row {row}, Column {column}')
    imgui.end_table()
  
  def query_test(self):
    if not hasattr(self, 'query_text'):
      self.query_text = 'text'
    
    _, self.query_text = imgui.input_text('query', self.query_text)

    imgui.text(f'is_item_hovered: {imgui.is_item_hovered()}')
    imgui.text(f'is_item_active: {imgui.is_item_active()}')
    imgui.text(f'is_item_focused: {imgui.is_item_focused()}')
    imgui.text(f'is_item_clicked: {imgui.is_item_clicked()}')
    imgui.text(f'is_item_visible: {imgui.is_item_visible()}')
    imgui.text(f'is_item_edited: {imgui.is_item_edited()}')
    imgui.text(f'is_item_deactivated: {imgui.is_item_deactivated()}')
    imgui.text(f'is_item_deactivated_after_edit: {imgui.is_item_deactivated_after_edit()}')
    imgui.text(f'is_item_toggled_open: {imgui.is_item_toggled_open()}')
    imgui.text(f'is_any_item_hovered: {imgui.is_any_item_hovered()}')
    imgui.text(f'is_any_item_active: {imgui.is_any_item_active()}')
    imgui.text(f'is_any_item_focused: {imgui.is_any_item_focused()}')
    imgui.text(f'get_item_id: {imgui.get_item_id()}')
    imgui.text(f'get_item_rect_min: ({imgui.get_item_rect_min()[0]}, {imgui.get_item_rect_min()[1]})')
    imgui.text(f'get_item_rect_max: ({imgui.get_item_rect_max()[0]}, {imgui.get_item_rect_max()[1]})')
    imgui.text(f'get_item_rect_size: ({imgui.get_item_rect_size()[0]}, {imgui.get_item_rect_size()[1]})')

    text = 'test'
    text_size = imgui.calc_text_size(text)
    imgui.text(f'calc_text_size("{text}"): ({text_size[0]}, {text_size[1]})')

  def ui_callback(self):
    imgui.begin('ui', flags=imgui.WindowFlags_AlwaysAutoResize)

    if imgui.begin_tab_bar('tab_bar'):
      if imgui.begin_tab_item('io'):
        self.io_test()
        imgui.end_tab_item()

      if imgui.begin_tab_item('style'):
        self.style_test()
        imgui.end_tab_item()

      if imgui.begin_tab_item('drawlist'):
        self.drawlist_test()
        imgui.end_tab_item()
      
      if imgui.begin_tab_item('demo'):
        self.demo_test()
        imgui.end_tab_item()

      if imgui.begin_tab_item('win'):
        self.win_test()
        imgui.end_tab_item()
      
      if imgui.begin_tab_item('param_stacks'):
        self.param_stacks_test()
        imgui.end_tab_item()
      
      if imgui.begin_tab_item('cursor'):
        self.cursor_test()
        imgui.end_tab_item()

      if imgui.begin_tab_item('text'):
        self.text_test()
        imgui.end_tab_item()
      
      if imgui.begin_tab_item('main_widget'):
        self.main_widget_test()
        imgui.end_tab_item()
      
      if imgui.begin_tab_item('image'):
        self.image_test()
        imgui.end_tab_item()
      
      if imgui.begin_tab_item('combo'):
        self.combo_test()
        imgui.end_tab_item()
      
      if imgui.begin_tab_item('drag'):
        self.drag_test()
        imgui.end_tab_item()
      
      if imgui.begin_tab_item('input_test'):
        self.input_text_test()
        imgui.end_tab_item()
      
      if imgui.begin_tab_item('color'):
        self.color_test()
        imgui.end_tab_item()
      
      if imgui.begin_tab_item('trees'):
        self.trees_test()
        imgui.end_tab_item()
      
      if imgui.begin_tab_item('menu'):
        self.menu_test()
        imgui.end_tab_item()
      
      if imgui.begin_tab_item('popup'):
        self.popup_test()
        imgui.end_tab_item()
      
      if imgui.begin_tab_item('table'):
        self.table_test()
        imgui.end_tab_item()
      
      if imgui.begin_tab_item('query'):
        self.query_test()
        imgui.end_tab_item()
      
      imgui.end_tab_bar()

    imgui.end()
  

test_ui = TestUI()
test_ui.spin()
