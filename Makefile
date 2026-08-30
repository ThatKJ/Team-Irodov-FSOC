.PHONY: configure build test run run-step2 run-step3 run-step4 run-step5 run-step6 run-step7 run-step8 clean format

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

run-step4: build
	./build/debug/step4_renderer_smoke

run-step5: build
	./build/debug/step5_detector_smoke

run-step6: build
	./build/debug/step6_pid_smoke

run-step7: build
	./build/debug/step7_closed_loop_smoke

run-step8: build
	./build/debug/step8_telemetry_smoke

format:
	find include src apps tests -type f \( -name '*.hpp' -o -name '*.cpp' \) -print0 | xargs -0 clang-format -i

clean:
	rm -rf build
