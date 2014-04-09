#!/bin/bash
# Linux users have to change $8 to $9


rm -r -f ../data/average.csv
LIMIT=150
INLIMIT=15
ITERATIONS=1

awk '
BEGIN { ARGV[1]="../data/g02_lab05data_01.csv"
	ARGC=2  	
	sum1=0
	average1=0
	sum2=0
	average2=0
	sum3=0
	min1=1000
	average3=0
	max=0
	sum4=0
	average4=0
	sum5=0
	average5=0
	average6=0
	diff=0
	k=1
	FS=","
	}	
{ 	if($1==k){
	if($3>max) {max=$3}
	if(min1>$3) {if($3!=0){min1=$3} }
	sum1+=$3
	sum2+=$4
	sum3+=$5
	sum4+=$6
	sum5+=$7
	}
	else{
	average1=sum1/15
	average2=sum2/15
	average3=sum3/15
	average4=sum4/15
	average5=sum5/15
	average6=average2+average3+average4
	diff=(max-min1)/2
	print k,average1,average2,average3,average4,average5,max,min1,average6,diff >> "../data/average.csv"
	sum1=0
	average1=0
	sum2=0
	average2=0
	sum3=0
	min1=1000
	average3=0
	max=0
	sum4=0
	average4=0
	sum5=0
	average5=0
	average6=0
	diff=0
	if($3>max) {max=$3}
	if(min1>$3) {min1=$3}
	sum1+=$3
	sum2+=$4
	sum3+=$5
	sum4+=$6
	sum5+=$7
	k=k+1
}
}
END { average1=sum1/15
	average2=sum2/15
	average3=sum3/15
	average4=sum4/15
	average5=sum5/15
	average6=average2+average3+average4
	diff=(max-min1)/2
	print k,average1,average2,average3,average4,average5,max,min1,average6,diff >> "../data/average.csv" }
'
	
