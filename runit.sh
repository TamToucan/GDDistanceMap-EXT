#!/bin/bash
DST=XXX
rm -fR $DST
mkdir XXX
find build -name "*.dll" | while read f
do
   echo cp $f $DST
   cp $f $DST
done

find build -name "*.exe" | while read f
do
   echo cp $f $DST
   cp $f $DST
done

cp distancemap/test/GRID.txt $DST
cd $DST
./testMain.exe
