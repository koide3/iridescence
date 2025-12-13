from setuptools import find_packages, setup
setup(
    cmake_source_dir=".",
    packages=['pyridescence_data'],
    package_dir={'pyridescence_data': 'data'}
)
