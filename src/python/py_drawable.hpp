#ifndef PYRIDESCENCE_PY_DRAWABLE_HPP
#define PYRIDESCENCE_PY_DRAWABLE_HPP

#include <pybind11/pybind11.h>
#include <memory>
#include <glk/drawable.hpp>
#include <glk/delayed_constructor.hpp>

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

/// @brief Check if the Python interpreter is alive (initialized and not finalizing).
///        The GIL must not be acquired from a non-Python thread after the interpreter started finalizing.
inline bool python_alive() {
#if PY_VERSION_HEX >= 0x030D0000
  return Py_IsInitialized() && !Py_IsFinalizing();
#else
  return Py_IsInitialized() && !_Py_IsFinalizing();
#endif
}

/// @brief Wrap a Python object so that it can be released in any thread (e.g., the AsyncLightViewer thread).
///        The GIL is acquired when the object is released. If the interpreter is already gone, the reference is leaked.
inline std::shared_ptr<py::object> make_gil_safe_object(py::object obj) {
  return std::shared_ptr<py::object>(new py::object(std::move(obj)), [](py::object* p) {
    if (python_alive()) {
      py::gil_scoped_acquire acquire;
      delete p;
    } else {
      p->release();
      delete p;
    }
  });
}

/// @brief Make a drawable safe to be released in any thread.
///        A Python-derived drawable (PyDrawable) holds a reference to its Python part, and releasing it requires the GIL.
///        Other drawables are returned as is.
inline std::shared_ptr<const glk::Drawable> make_gil_safe_drawable(const std::shared_ptr<const glk::Drawable>& drawable) {
  if (!dynamic_cast<const PyDrawable*>(drawable.get())) {
    return drawable;
  }

  auto holder = new std::shared_ptr<const glk::Drawable>(drawable);
  return std::shared_ptr<const glk::Drawable>(drawable.get(), [holder](const glk::Drawable*) {
    if (python_alive()) {
      py::gil_scoped_acquire acquire;
      delete holder;
    } else {
      // Leak the drawable because the Python part cannot be released after the interpreter is finalized
    }
  });
}

/// @brief Create a DelayedConstructor that calls cls(*args, **kwargs) upon the first draw() in the rendering thread.
inline std::shared_ptr<glk::DelayedConstructor> make_py_delayed_constructor(const py::object& cls, const py::args& args, const py::kwargs& kwargs) {
  if (!PyCallable_Check(cls.ptr())) {
    throw py::type_error("cls must be callable");
  }

  // (cls, args, kwargs) may be released in the rendering thread
  auto call = make_gil_safe_object(py::make_tuple(cls, args, kwargs));

  return std::make_shared<glk::DelayedConstructor>([call]() -> glk::Drawable::ConstPtr {
    py::gil_scoped_acquire acquire;
    try {
      const py::tuple t = call->cast<py::tuple>();
      py::object obj = t[0](*t[1], **t[2]);
      const auto drawable = obj.cast<std::shared_ptr<glk::Drawable>>();
      return make_gil_safe_drawable(drawable);
    } catch (const py::error_already_set& e) {
      // error_already_set must not escape the GIL scope
      throw std::runtime_error(e.what());
    } catch (const py::cast_error&) {
      throw std::runtime_error("the constructed object is not a glk.Drawable");
    }
  });
}

#endif
