from setuptools import find_packages, setup
setup(
    name="pyridescence",
    version="0.1.6",
    author="k.koide",
    author_email="k.koide@aist.go.jp",
    description="A 3D visualization library for rapid proto-typing",
    license="MIT",
    cmake_source_dir=".",
    zip_safe=False,
    packages=['pyridescence_data'],
    package_dir={'pyridescence_data': 'data'},
    python_requires=">=3.7",
    include_package_data=True,
)
