#!/bin/bash

perf record -o g1.dat -g bin/cs296_base 10
perf script -i g1.dat | python gprof2dot.py -f perf | dot -Tpng -o g1.png
perf record -o g2.dat -g bin/cs296_base 100
perf script -i g2.dat | python gprof2dot.py -f perf | dot -Tpng -o g2.png
perf record -o g3.dat -g bin/cs296_base 1000 
perf script -i g3.dat | python gprof2dot.py -f perf | dot -Tpng -o g3.png
