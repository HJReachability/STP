

.PHONY: all 3rdparty modules SeqPP time time_chunk_sizes \
	iprofiler valgrind massif massif_chunk_sizes helgrind \
	unittest sampletest test clean cleanall

all: SeqPP
	cd samples && make all
	cd unittests && make all

SeqPP:
	cd SeqPP && make all

time: SeqPP
	cd samples && make time

sampletest: SeqPP
	cd samples && make test

test: SeqPP sampletest

clean:
	cd SeqPP && make clean
	cd samples && make clean

cleanall:
	cd SeqPP && make clean
	cd samples && make clean
