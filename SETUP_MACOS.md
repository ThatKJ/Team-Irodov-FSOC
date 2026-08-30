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

## 5. OpenCV — only when Step 4/5 begins

```bash
brew install opencv
```

Do not couple OpenCV windows/rendering to camera physics or PID logic. OpenCV is an observation/rendering adapter.

## 6. No Python setup

Do not create `.venv`, run `pip install`, or add `pyproject.toml`. If an AI agent suggests those commands, reject that change because this repository is C++20-first.
