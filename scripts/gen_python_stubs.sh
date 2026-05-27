#!/bin/bash

pushd .

dst_path=$(realpath src/python/stubs)

cd build
python -m pybind11_stubgen -o $dst_path pyridescence

popd

