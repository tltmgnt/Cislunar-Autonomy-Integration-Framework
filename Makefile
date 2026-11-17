CXX ?= g++
CC ?= gcc
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra -Wpedantic
CPPFLAGS = -Iinclude
CORE = src/core.cpp src/c_api.cpp
.PHONY: all test clean
all: build/cislunar_sim
build:
	mkdir -p build
build/cislunar_sim: $(CORE) app/sim.cpp include/cislunar/core.hpp include/cislunar/c_api.h | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(CORE) app/sim.cpp -o $@
build/core_tests: $(CORE) tests/core_tests.cpp include/cislunar/core.hpp include/cislunar/c_api.h | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(CORE) tests/core_tests.cpp -o $@
build/c_api_test.o: tests/c_api_test.c include/cislunar/c_api.h | build
	$(CC) $(CPPFLAGS) -std=c11 -Wall -Wextra -Wpedantic -c $< -o $@
build/c_api_test: $(CORE) build/c_api_test.o
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(CORE) build/c_api_test.o -o $@
test: all build/core_tests build/c_api_test
	./build/core_tests
	./build/c_api_test
	python3 tests/integration.py ./build/cislunar_sim
clean:
	rm -rf build
