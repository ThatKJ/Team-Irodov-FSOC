.PHONY: configure build test run clean format

configure:
	cmake --preset debug

build:
	cmake --build --preset debug

test: build
	ctest --preset debug

run: build
	./build/debug/step1_math_smoke

format:
	find include src apps tests -type f \( -name '*.hpp' -o -name '*.cpp' \) -print0 | xargs -0 clang-format -i

clean:
	rm -rf build
