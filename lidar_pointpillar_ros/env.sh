#!/usr/bin/env bash


set -e

# cmake install
# wget https://cmake.org/files/v3.23/cmake-3.23.1.tar.gz
# tar -zxvf cmake-3.23.1.tar.gz
cd docker/cmake-3.23.1 && ./configure && make -j32 && sudo make install

#cuda install
cd ../
sudo sh cuda_11.3.0_465.19.01_linux.run

# cudnn install  version == 8.2.1
sudo cp cuda/include/cudnn.h /usr/local/cuda-11.3/include/ 
sudo cp cuda/include/cudnn_version.h /usr/local/cuda-11.3/include/
sudo cp cuda/lib64/libcudnn* /usr/local/cuda-11.3/lib64/ 
sudo chmod a+r /usr/local/cuda-11.3/include/cudnn.h
sudo chmod a+r /usr/local/cuda-11.3/lib64/libcudnn*

# TensorRT install
sudo cp -r  TensorRT-8.2.1.8/ /opt/
sudo cp -r /opt/TensorRT-8.2.1.8/lib/lib* /usr/lib/x86_64-linux-gnu/
sudo cp -r /opt/TensorRT-8.2.1.8/include/* /usr/include/x86_64-linux-gnu/

#export PATH="/usr/local/cuda-11.3/bin:$PATH"
#export LD_LIBRARY_PATH="/usr/local/cuda-11.3/lib64:$LD_LIBRARY_PATH"
# export LD_LIBRARY_PATH=$PATH:/opt/TensorRT-8.2.1.8/lib:$LD_LIBRARY_PATH
# export LIBRARY_PATH=$PATH:/opt/TensorRT-8.2.1.8/lib:$LIBRARY_PATH
