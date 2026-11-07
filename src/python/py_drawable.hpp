#ifndef PYRIDESCENCE_PY_DRAWABLE_HPP
#define PYRIDESCENCE_PY_DRAWABLE_HPP

#include <pybind11/pybind11.h>
#include <glk/drawable.hpp>

namespace py = pybind11;

// Trampoline class to allow implementing glk::Drawable in Python
class PyDrawable : public glk::Drawable {
public:
  void draw(glk::GLSLShader& shader) const override {
    // PYBIND11_OVERRIDE is not used here because it passes lvalue references to Python as copies,
    // and destroying a copy of GLSLShader deletes the shader program of the viewer.
    py::gil_scoped_acquire acquire;
    py::function override = py::get_override(static_cast<const glk::Drawable*>(this), "draw");
    if (override) {
      override(py::cast(shader, py::return_value_policy::reference));
      return;
    }
    glk::Drawable::draw(shader);
  }
};

// Keep the Python part of a Python-derived drawable alive while C++ holds the returned pointer.
// Without this, the Python override is lost once the Python object is garbage collected.
inline std::shared_ptr<glk::Drawable> keep_python_drawable_alive(const py::object& obj) {
  auto drawable = obj.cast<std::shared_ptr<glk::Drawable>>();
  if (!dynamic_cast<PyDrawable*>(drawable.get())) {
    return drawable;
  }

  auto holder = new py::object(obj);
  return std::shared_ptr<glk::Drawable>(drawable.get(), [holder](glk::Drawable*) {
    // Leak the object if the interpreter has already been finalized
    if (!Py_IsInitialized()) {
      return;
    }
    py::gil_scoped_acquire acquire;
    delete holder;
  });
}

#endif
