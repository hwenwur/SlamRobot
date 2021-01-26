.PHONY: all

all: read write

%: %.cpp
	g++ -std=c++11 $^ -o $@

clean:
	rm -rf ./read ./write
