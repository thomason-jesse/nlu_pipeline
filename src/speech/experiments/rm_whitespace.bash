#!/bin/sh
#This script removes whitespace in fileids files. 

CURRENT_DIR=$( pwd )

files="$CURRENT_DIR/k-folds/4_fold/*/recordings/test/ids*.fileids"

for i in $files; do
  sed '/^$/d' $i > $i.out
  mv  $i.out $i
done
