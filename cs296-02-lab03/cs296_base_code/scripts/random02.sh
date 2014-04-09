#!/bin/bash
# Linux users have to change $8 to $9


rm -r -f ../data/random02_avg.csv
LIMIT=1500
INLIMIT=150

awk '
BEGIN { ARGV[1]="../data/g02_lab05data_02.csv"
	ARGC=2  	
	sum1=0
	average1=0
	k=1
	FS=","
	}	
{ 	if($1==k){
	sum1+=$3 
	} else{
		average1=sum1/150
		print k,average1 >> "../data/random02_avg.csv" 
		sum1=0
		average1=0
		sum1+=$3
		k+=1
	}		
}
END{ average1=sum1/150
		print k,average1 >> "../data/random02_avg.csv" 
}
'
