#!/bin/sh
d=$(ssh ros@ros-pc4 "date +%s" 2>&1)

sudo date -s @$d

echo "Datum von 192.168.64.211 geholt !"
