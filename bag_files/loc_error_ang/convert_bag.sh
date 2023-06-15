#!/bin/bash

bag_files=$(find -name "*.bag")

for files in ${bag_files[@]}
do
  f=$(basename $files .bag)
  $(rostopic echo -b $files -p /ang_error > $f.csv)
  $(mv $files converted_bag)
done
