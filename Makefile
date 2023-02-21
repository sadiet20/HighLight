CC=g++ -std=c++11 -g 

all: parse_files_pico

parse_files_pico: parse_files_pico.cpp
	$(CC) parse_files_pico.cpp -o parse_files_pico

clean:
	rm -f *.o parse_files_pico
