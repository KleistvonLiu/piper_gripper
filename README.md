# piper_gripper_cpp_repo

Standalone C++ gripper module extracted from `piper_sdk`.

## Build

```bash
cmake -S . -B build -DPIPER_GRIPPER_BUILD_EXAMPLES=ON
cmake --build build -j4
```

## Run Examples

```bash
./build/test_gripper_cpp
./build/test_gripper_effort_cpp
```

## Tests

```bash
ctest --test-dir build --output-on-failure
```
