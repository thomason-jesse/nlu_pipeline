#!/bin/bash

if [ -e logs.txt ] 
then
	rm logs.txt
fi

for dir in *; do
	if [ -d $dir ] 
	then
		OFD=$dir/offline_data
		
		for dir1 in $OFD/*; do
			if [ -d $dir1 ]
			then
				echo $dir1/logs:
				echo ""

				for txtFile in $dir1/logs/*.txt; do
					python concatTxtFile.py $txtFile
				done
			fi
		done
	fi
done
