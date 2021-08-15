#!/bin/bash

# Shell script for comparing frandom with urandom
# May be executed as a non-root user

if [ ! -e /dev/frandom ]; then
    echo "It looks like you need to insmod frandom. (/dev/frandom file missing)";
    exit 1;
fi

if [ ! -e /dev/urandom ]; then
    echo "/dev/urandom does not exist. Weird...";
    exit 1;
fi

# Now verify that our generators work:
if ! head --bytes=1k /dev/frandom > /dev/null; then
    echo -e "\n/dev/frandom doesn't work! See the INSTALL file.";
    exit 1;
fi

if ! head --bytes=1k /dev/urandom > /dev/null; then
    echo -e "\n/dev/urandom doesn't work! (This is weird...)";
    exit 1;
fi

# We don't want the test to take too long on slow computers, and it's
# meaningless if it goes to fast. Solution: Use bogomips as an
# indication of how fast or slow the computer is.

bogomips=`grep -i bogomips /proc/cpuinfo`;
bogomips=${bogomips##*:}; # Remove everything before ":"
bogomips=${bogomips%%.*}; # Remove anything after "."
urandomkbytes=$((bogomips*5));
frandomkbytes=$((bogomips*50));

failed=0

echo "Running tests... This may take up to a minute"

(
echo -e "Results of frandom vs. urandom test:"
echo -e "====================================\n"
date
echo -e "\nurandom created $urandomkbytes kBytes of data"
echo -e "frandom created $frandomkbytes kBytes of data\n"
echo -e "Note that frandom did 10 times as much data!\n"

echo -e "head --bytes=${frandomkbytes}k /dev/frandom > /dev/null"
time head --bytes=${frandomkbytes}k /dev/frandom > /dev/null

echo -e "\nhead --bytes=${urandomkbytes}k /dev/urandom > /dev/null"
time head --bytes=${urandomkbytes}k /dev/urandom > /dev/null

echo -e "\nPlatform info:";
uname -s -r
cat /proc/cpuinfo

) > frandom-res.txt 2>&1

echo -e "Test finished successfully. See frandom-res.txt for results";
