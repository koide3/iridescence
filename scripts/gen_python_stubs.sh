#!/bin/bash
# Regenerate type stubs for the installed pyridescence package (pip install . first).
set -e

dst_path=$(realpath src/python)
tmp_path=$(mktemp -d)

python -m pybind11_stubgen -o $tmp_path pyridescence._pyridescence

# Stubs are generated for the extension module (pyridescence/_pyridescence) but describe the public package (pyridescence)
find $tmp_path -name "*.pyi" -exec sed -i 's/pyridescence\._pyridescence/pyridescence/g' {} +
# __init__.pyi is maintained by hand (it mirrors src/python/pyridescence/__init__.py)
rm $tmp_path/pyridescence/_pyridescence/__init__.pyi
cp -r $tmp_path/pyridescence/_pyridescence/. $dst_path/pyridescence/

rm -rf $tmp_path
