#!/usr/bin/python3
from collections import deque
import numpy
from pyridescence import *

# ImPlot flags and enums (see implot.h)
ImPlotFlags_Equal = 1 << 8
ImPlotAxisFlags_AutoFit = 1 << 11
ImPlotLineFlags_Segments = 1 << 10
ImPlotLineFlags_Loop = 1 << 11
ImPlotLineFlags_Shaded = 1 << 14
ImPlotStairsFlags_PreStep = 1 << 10
ImPlotStairsFlags_Shaded = 1 << 11
ImPlotHistogramFlags_Cumulative = 1 << 11
ImPlotHistogramFlags_Density = 1 << 12
ImPlotLegendFlags_Horizontal = 1 << 5
ImPlotLocation_SouthWest = (1 << 1) | (1 << 2)
ImAxis_X1 = 0

viewer = guik.viewer()

xs = numpy.arange(0.0, 2.0 * numpy.pi, 0.1)
ys_sin = numpy.sin(xs)
ys_cos = numpy.cos(xs)

# ---- Basic plotting ----
# If a plot name contains "/", the string before the slash is recognized as a group name.
# Plots with the same group name are displayed in the same tab.
viewer.setup_plot('basic/curves_y', 1024, 256)
viewer.update_plot_line('basic/curves_y', 'sin', ys_sin)  # When only Y values are given, X values become index IDs
viewer.update_plot_stairs('basic/curves_y', 'sin_stairs', ys_sin)

viewer.setup_plot('basic/curves_xy', 1024, 256)
viewer.update_plot_line('basic/curves_xy', 'sin', xs, ys_sin)
viewer.update_plot_stairs('basic/curves_xy', 'sin_stairs', xs, ys_sin)

# Equal axis scales and a closed loop
viewer.setup_plot('basic/circle', 512, 512, ImPlotFlags_Equal)
viewer.update_plot_line('basic/circle', 'circle', numpy.cos(xs), numpy.sin(xs), ImPlotLineFlags_Loop)

# ---- Plot types and flags ----
# Line plots with flags (see ImPlotLineFlags in implot.h)
viewer.setup_plot('styles/lines', 1024, 256)
viewer.update_plot_line('styles/lines', 'shaded', xs, ys_sin, ImPlotLineFlags_Shaded)
viewer.update_plot_line('styles/lines', 'segments', xs, ys_cos, ImPlotLineFlags_Segments)
# Legend location and flags (see ImPlotLocation and ImPlotLegendFlags)
viewer.setup_legend('styles/lines', ImPlotLocation_SouthWest, ImPlotLegendFlags_Horizontal)

# Scatter and stairs plots
viewer.setup_plot('styles/scatter_stairs', 1024, 256)
viewer.update_plot_scatter('styles/scatter_stairs', 'scatter', xs, ys_sin)
viewer.update_plot_stairs('styles/scatter_stairs', 'stairs_shaded', xs, ys_cos, ImPlotStairsFlags_Shaded)
viewer.update_plot_stairs('styles/scatter_stairs', 'stairs_prestep', xs, ys_sin, ImPlotStairsFlags_PreStep)

# Spiral
ts = numpy.arange(0.0, 6.0 * numpy.pi, 0.05)
viewer.setup_plot('styles/spiral', 512, 512, ImPlotFlags_Equal)
viewer.update_plot_line('styles/spiral', 'spiral', ts * numpy.cos(ts), ts * numpy.sin(ts))
viewer.update_plot_scatter('styles/spiral', 'samples', ts[::10], 15.0 * numpy.sin(ts[::10]))

# ---- Histograms ----
rng = numpy.random.default_rng(0)
gauss_x = rng.normal(0.0, 1.0, 5000)
gauss_y = 0.5 * gauss_x + 0.5 * rng.normal(0.0, 1.0, 5000)

# 1D histograms. The number of bins can be a positive number or an automatic binning method (negative value, see ImPlotBin).
viewer.setup_plot('histogram/1d', 1024, 256)
viewer.update_plot_histogram('histogram/1d', 'x (auto bins)', gauss_x)
viewer.update_plot_histogram('histogram/1d', 'y (20 bins)', gauss_y, x_bins=20)

viewer.setup_plot('histogram/1d_density', 1024, 256)
viewer.update_plot_histogram('histogram/1d_density', 'density', gauss_x, x_bins=50, x_range=[-4.0, 4.0], histogram_flags=ImPlotHistogramFlags_Density)
viewer.update_plot_histogram('histogram/1d_density', 'cumulative', gauss_x, x_bins=50, x_range=[-4.0, 4.0], histogram_flags=ImPlotHistogramFlags_Density | ImPlotHistogramFlags_Cumulative)

# 2D histogram
viewer.setup_plot('histogram/2d', 512, 512)
viewer.update_plot_histogram('histogram/2d', 'xy', gauss_x, gauss_y, x_bins=50, y_bins=50)

# ---- Realtime plots ----
# AutoFit keeps the axes fitted to the data that are updated every frame
viewer.setup_plot('realtime/signal', 1024, 200, 0, ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_AutoFit)
viewer.setup_plot('realtime/noise', 1024, 200, 0, ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_AutoFit)
# Axes with the same link ID are synchronized (try zooming the X axis after disabling the auto-fitting from the right click menu)
viewer.link_plot_axis('realtime/signal', 0, ImAxis_X1)
viewer.link_plot_axis('realtime/noise', 0, ImAxis_X1)

# Display order of the plot groups (tabs)
viewer.setup_plot_group_order('basic', 0)
viewer.setup_plot_group_order('styles', 1)
viewer.setup_plot_group_order('histogram', 2)
viewer.setup_plot_group_order('realtime', 3)

running = True
reset = False


def plot_ui():
  global running, reset
  imgui.begin('plot control', True, imgui.WindowFlags_AlwaysAutoResize)
  _, running = imgui.checkbox('update realtime plots', running)
  if imgui.button('fit all plots'):
    viewer.fit_all_plots()
  imgui.same_line()
  if imgui.button('reset realtime data'):
    reset = True
  imgui.end()


viewer.register_ui_callback('plot_ui', plot_ui)

# Keep only the latest data
max_num_data = 500
stamps, signal, envelope, noise = (deque(maxlen=max_num_data) for _ in range(4))

t = 0.0
while viewer.spin_once():
  if reset:
    reset = False
    for data in (stamps, signal, envelope, noise):
      data.clear()
  if not running:
    continue

  t += imgui.get_io().delta_time
  stamps.append(t)
  signal.append(numpy.sin(t * 2.0) * numpy.exp(-0.2 * (t % 10.0)))
  envelope.append(numpy.exp(-0.2 * (t % 10.0)))
  noise.append(0.1 * rng.normal())

  # Plots with the same name and label are overwritten
  viewer.update_plot_line('realtime/signal', 'signal', stamps, signal)
  viewer.update_plot_line('realtime/signal', 'envelope', stamps, envelope)
  viewer.update_plot_scatter('realtime/noise', 'noise', stamps, noise)
