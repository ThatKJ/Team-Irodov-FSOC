.PHONY: configure build test run run-step2 run-step3 clean format

configure:
	cmake --preset debug

build:
	cmake --build --preset debug

test: build
	ctest --preset debug

run: build
	./build/debug/step1_math_smoke

run-step2: build
	./build/debug/step2_trajectory_smoke

run-step3: build
	./build/debug/step3_observation_smoke

format:
	find include src apps tests -type f \( -name '*.hpp' -o -name '*.cpp' \) -print0 | xargs -0 clang-format -i

clean:
	rm -rf build
