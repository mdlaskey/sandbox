#!/bin/bash
/usr/local/caffe/tools/extra/parse_log.py ./$1/logs/train.log ./$1/logs/ && 
python plot.py --net $1 && 
cd ..
