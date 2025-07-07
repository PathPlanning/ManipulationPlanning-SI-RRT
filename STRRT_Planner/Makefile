.PHONY: clean build_debug build_release run_tests

clean:
	rm -r ./build

build_debug:
	mkdir -p build
	cmake -B build -DCMAKE_BUILD_TYPE=Debug 
	cmake --build build -j 128 --config Debug

build_release:
	mkdir -p build
	cmake -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_FLAGS_RELEASE="-march=native -mtune=native -O3 -no-pie" \
        -DCMAKE_CXX_FLAGS_RELEASE="-march=native -mtune=native -O3 -no-pie"  -DCMAKE_CXX_FLAGS_RELEASE="-march=native -mtune=native -O3 -no-pie" 
	cmake --build build -j 128 --config Release
