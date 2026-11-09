#ifndef PYRIDESCENCE_PY_DRAWABLE_HPP
#define PYRIDESCENCE_PY_DRAWABLE_HPP

#include <pybind11/pybind11.h>
#include <glk/drawable.hpp>

namespace py = pybind11;

// Trampoline class to allow implementing glk::Drawable in Python.
// With py::smart_holder, trampoline_self_life_support keeps the Python part of a Python-derived drawable
// alive while C++ holds a std::shared_ptr to it, so the Python override is not lost after garbage collection.
// Note that every class in the glk::Drawable hierarchy must be bound with py::smart_holder.
class PyDrawable : public glk::Drawable, public py::trampoline_self_life_support {
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

#endif
