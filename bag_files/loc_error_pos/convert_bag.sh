#!/bin/bash

bag_files=$(find -name "*.bag")

for files in ${bag_files[@]}
do
  f=$(basename $files .bag)
  $(rostopic echo -b $files -p /position_err > $f.csv)
  $(mv $files converted_bag)
done
