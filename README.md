<a href="https://107-systems.org/"><img align="right" src="https://raw.githubusercontent.com/107-systems/.github/main/logo/107-systems.png" width="15%"></a>
:floppy_disk: `libdynamixelplusplus`
====================================
[![Smoke test status](https://github.com/107-systems/libdynamixelplusplus/actions/workflows/smoke-test.yml/badge.svg)](https://github.com/107-systems/libdynamixelplusplus/actions/workflows/smoke-test.yml)
[![Spell Check status](https://github.com/107-systems/libdynamixelplusplus/actions/workflows/spell-check.yml/badge.svg)](https://github.com/107-systems/libdynamixelplusplus/actions/workflows/spell-check.yml)

A comfortable modern C++17 wrapper for the Robotis [DynamixelSDK](https://github.com/ROBOTIS-GIT/DynamixelSDK) [v3.7.60](https://github.com/ROBOTIS-GIT/DynamixelSDK/releases/tag/3.7.60).

### How-to-build
```bash
git clone https://github.com/107-systems/libdynamixelplusplus && cd libdynamixelplusplus
mkdir build && cd build
cmake .. && make
```
or
```bash
cmake -DBUILD_EXAMPLES=ON .. && make
```
