from setuptools import find_packages, setup

with open("README.md", "r", encoding="utf-8") as fh:
    long_description = fh.read()

setup(
    name="pyridescence",
    version="1.0.0",
    author="Kenji Koide",
    author_email="k.koide@aist.go.jp",
    description="3D visualization library for rapid prototyping of 3D algorithms",
    long_description=long_description,
    long_description_content_type="text/markdown",
    license="MIT",
    cmake_source_dir=".",
    packages=['pyridescence_data', 'pyridescence_data.texture', 'pyridescence_data.shader', 'pyridescence_data.models'],
    package_dir={'pyridescence_data': 'data'},
    include_package_data=True,
    python_requires=">=3.7",
    install_requires=["numpy"],
)
