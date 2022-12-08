#!/bin/bash

arr=("mode" "mean" "median")
win=("5" "10" "15" "20" "50" "100" "200")

for filter in "${arr[@]}"
do
    for window in "${win[@]}"
    do
        echo "Filter: $filter, Window: $window"
    done
done