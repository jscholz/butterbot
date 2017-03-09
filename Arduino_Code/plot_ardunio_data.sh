#!/bin/bash

# This script will create a named pipe (fifo) for passing commands to gnuplot.
# by creating this pipe once (and we can check for it using if -p) we can keep 
# feeding stdin to a running gnuplot instance, which in this case is just a plot
# command followed by a bunch of rep (replot) requests
#
# The script is also configured to be flexible about the number of traces in the
# datfile, and will generate the appropriate number of plot calls by parsing the
# 2nd to last line of the file (since the last line might not be fully written)
# Expects data for each trace in columns:
#    205 53 330
#    205 53 329
#    205 53 330
#    205 52 330
#    
# Jonathan Scholz
# December 2009

#TODO:
#add cmd flags for the path, and perhaps also the datfile and whether or not to overwrite it

arduino_path=/dev/tty.usbserial-A600ahdX

# Cleanup from previous instances 
killall tail arduino-serial gnuplot
rm -f cmdfifo

# start reading data into file
./arduino-serial -p $arduino_path -u z > range.dat &
sleep 2
numcols=`tail -n 2 range.dat | head -n 1 | tr ' ' '\n' | wc -l`

# Attach a fifo to a gnuplot instance
rm -f cmdfifo
mkfifo cmdfifo
tail -f cmdfifo | /opt/local/bin/gnuplot &

# Initialize gnuplot
cat <<EOF > cmdfifo
#set term x11 # x11 or aqua
#set multiplot
#set xrange [400:700]
#set yrange [-50:1200]
#set zrange [400:700]
EOF

# Generate the appropriate number of plot commands based on the number of columns of data detected
echo -n "plot " > cmdfifo
for (( i=1; i<$((numcols+1)); i++ ))
do 
    echo -n '"< tail -200 range.dat"' using $i with lines > cmdfifo
    if [ $i -lt $numcols ]
    then 
        echo -n ", " > cmdfifo
    else
        echo " " > cmdfifo
    fi
done

# Replot at fixed interval
sleepdur=0.05
while [ 1 ]
do
    echo "rep" > cmdfifo
    sleep $sleepdur
done
