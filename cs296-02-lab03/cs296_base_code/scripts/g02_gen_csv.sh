#!/bin/bash
# Linux users have to change $8 to $9

LIMIT=150
rm -r -f ../data/g02_lab05data_01.csv
INLIMIT=15
ITERATIONS=1
while [ $ITERATIONS -le "$LIMIT" ]
do
RERUN=1 
while [ $RERUN -le "$INLIMIT" ]
do
awk ' 
BEGIN { 
	ARGV[1]="../data/g02out-'$ITERATIONS'-'$RERUN'.txt"
	ARGC=2 
	ORS = ","
	OFS = ","
}
{
if($1 == "Number"){
	print $4,'$RERUN' >> "../data/g02_lab05data_01.csv"
} else if($4 == "step"){
	print $6 >> "../data/g02_lab05data_01.csv"
} else if($4 == "collisions"){
	print $6 >> "../data/g02_lab05data_01.csv"
} else if($4 == "position" ){
	print $7 >> "../data/g02_lab05data_01.csv"
} else if($4 == "velocity" ){
	print $7 >> "../data/g02_lab05data_01.csv"
} else if($2 == "loop" ){
	ORS="\n"
	print $5 >> "../data/g02_lab05data_01.csv"
}
} 
END { }
' 
 RERUN=$(($RERUN+1))
done
 ITERATIONS=$(($ITERATIONS+1))
done
