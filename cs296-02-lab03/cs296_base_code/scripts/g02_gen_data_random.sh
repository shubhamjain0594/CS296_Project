#!/bin/bash
# random data generation

LIMIT=150
a=1
rm -r -f ../data/g02_lab05data_random.csv
while [ $a -le "$LIMIT" ]
do
b=1
INLIMIT=15
while [ $b -le "$INLIMIT" ]
do
wax=$(( ( RANDOM % 15 ) + 1 ))
row=$((($a-1)*$INLIMIT+$wax))
sed -n "$row p" < ../data/g02_lab05data_02.csv >> ../data/g02_lab05data_random.csv
b=$(($b+1))
done
a=$(($a+1))
done