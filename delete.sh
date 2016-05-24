#!/bin/bash

for dir in */;
do
	cd $dir;
	rm -rf bin build;
	cd ../;
done
