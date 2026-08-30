# macOS Setup — C++20

## 1. Install Apple command-line compiler

```bash
xcode-select --install
clang++ --version
```

## 2. Install build tools

```bash
brew install cmake ninja
cmake --version
ninja --version
```

If Homebrew is not installed, install it from the official Homebrew site first.

## 3. Configure, compile, and test

From the repository root:

```bash
cmake --preset debug
cmake --build --preset debug
ctest --preset debug
./build/debug/step1_math_smoke
```

Expected test result:

```text
100% tests passed, 0 tests failed
```

## 4. Optional developer tools

```bash
brew install clang-format llvm
```

## 5. OpenCV — required from Step 4 onward

```bash
brew install opencv
```

The build uses `find_package(OpenCV)` (components `core`, `imgcodecs`) — no Homebrew
paths are hardcoded. `FSOC_ENABLE_OPENCV` is `AUTO` by default: Steps 1–3 still configure,
build, and pass without OpenCV, and CMake prints a one-line notice if it is missing. Set
`-DFSOC_ENABLE_OPENCV=ON` to make it mandatory, `OFF` to never look for it.

After installing, run the normal workflow:

```bash
cmake --preset debug
cmake --build --preset debug
ctest --preset debug
./build/debug/step4_renderer_smoke      # writes generated/*.png headlessly
```

Do not couple OpenCV windows/rendering to camera physics or PID logic. OpenCV lives only
in the `fsoc_render` library (the image/perception boundary); `fsoc_core` stays OpenCV-free.

## 6. No Python setup

Do not create `.venv`, run `pip install`, or add `pyproject.toml`. If an AI agent suggests those commands, reject that change because this repository is C++20-first.
