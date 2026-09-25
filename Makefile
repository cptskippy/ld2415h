CXX ?= g++
CXXFLAGS ?= -std=c++17 -Wall -Wextra -Werror -O2 -Iinclude

all: test

test: test/test_ld2415h
	./test/test_ld2415h

test/test_ld2415h: test/test_ld2415h.cpp src/LD2415H.cpp include/LD2415H.h
	$(CXX) $(CXXFLAGS) $< src/LD2415H.cpp -o $@

clean:
	rm -f test/test_ld2415h

.PHONY: all test clean
