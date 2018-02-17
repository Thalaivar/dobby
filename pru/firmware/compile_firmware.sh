#!/bin/bash

echo "Enter the file name to compile:"
read filename

pasm -b $filename.p
mv ${filename}.bin ../bin/
echo "PRU firmware succesfully compiled! Check ../lib/"

rm *.bin
