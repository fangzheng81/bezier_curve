[![Build Status](https://travis-ci.org/xmba15/bezier_curve.svg?branch=master)](https://travis-ci.org/xmba15/bezier_curve/builds)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

# Generic Bezier Curve Library #
- Generic Bezier Curve Library that supports control points of arbitary dimension numbers. (2D and 3D for normal, curvature calculations).
- Class of control point Container is built on top of Eigen Matrix; but can possibly be replaced with user-defined Point Container Class (which is quite a tedious task if starting from 0).

## Dependencies ##
- [Eigen](http://eigen.tuxfamily.org) : **Current build with cmake was tested on Ubuntu 18.04. Build on lower version of Ubuntu OS might require modification of CMake to find Eigen OR manually build latest Eigen from source.**
- [Doxygen](http://www.doxygen.nl/index.html) : Only used for building documentations. The lack of doxygen does not affect the building of this library.

## How to Build ##
```bash
cd /path/to/bezier_curve
mkdir build && cd build
cmake ../ && cmake --build .
```

### Build Options ###
 - WITH_DEBUG=ON/OFF(default: OFF) : whether to enable debug
 - BUILD_EXAMPLES=ON/OFF(default: OFF) : whether to enable build examples
 - BUILD_DOC=ON/OFF(default: ON) : whether to build doc (by doxygen) or not
## General Usage ##
See [Examples](examples)

## Examples ##
### 2d points ##

Example for 5-degree bezier curve of the following 2d control points:

[Source Code](./examples/Ex2DPoints.cpp)

<p align="center">
    <img src="./docs/images/2dpoints.png", width="640">
</p>

### 3d points ##

Example for 9-degree bezier curve of the following 3d control points:

[Source Code](./examples/Ex3DPoints.cpp)

<p align="center">
    <img src="./docs/images/3dpoints.png", width="640">
</p>

## Acknowledgement ##
- This library is inspired by the following [Bezier library](https://github.com/oysteinmyrmo/bezier), which supports only 2D space (as of 21st June, 2019)
