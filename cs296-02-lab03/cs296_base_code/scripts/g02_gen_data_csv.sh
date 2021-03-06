#!/bin/bash
# g02_gen_data.sh

LIMIT=150 # upper limit for number of iterations

a=1
rm -r -f ../data/g02_lab05data_02.csv

while [ $a -le "$LIMIT" ]
do
b=1
INLIMIT=15
while [ $b -le "$INLIMIT" ] 
do 
 export c=$(../bin/cs296_base "$a") 
 set -- $c
 var=$(echo $c |
 awk ' 
BEGIN {  
	ORS = "\n"
	OFS = ","
}
{
	print $4,'$b',$10,$17,$25,$33,$39 >> "../data/g02_lab05data_02.csv"
} 
END { ORS="," }
' 
)
$var
 b=$(($b+1))
done
 a=$(($a+1))
done	
