.PHONY: help
help:
	@echo "make cpp|py"

.PHONY: cpp
cpp:
	@echo "Building C++ documentation..."
	doxygen Doxyfile doc_cpp

.PHONY: py
py:
	@echo "Building Python documentation..."
	mkdir -p build && cd build && cmake ../../ -DBUILD_PYTHON_BINDINGS=ON && make -j
	sphinx-build -b html . ./doc_py/

.PHONY: all
all: cpp py
	@echo "All documentation built."
