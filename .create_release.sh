#!/bin/bash
####################################################
## Stand-alone example create release script
## Murilo M. Marinho (www.murilomarinho.info)
####################################################

release_folder=tro2022_adaptivecontrol_example

# Make release folder
rm -rf ${release_folder}
mkdir ${release_folder}
cd ${release_folder}

# Copy files
cp -r ../bin .
cp -r ../lib .
cp ../.run.sh run_example.sh
cp ../*ttt .

# Compress
cd ..
tar -cJf tro2022_adaptivecontrol_example.tar.xz tro2022_adaptivecontrol_example/
