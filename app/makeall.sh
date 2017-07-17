#!/bin/sh
#
# make all cui applications by gcc
#

echo; echo % ../lib/iers/gcc
cd ../lib/iers/gcc
make $1
if [ $? -ne 0 ]; then
    exit 3
fi
cd ../../../app/

echo; echo % pos2kml/gcc
cd pos2kml/gcc
make $1
if [ $? -ne 0 ]; then
    exit 3
fi
cd ../..

echo; echo % str2str/gcc
cd str2str/gcc
make $1
if [ $? -ne 0 ]; then
    exit 3
fi
cd ../..

echo; echo % rnx2rtkp/gcc
cd rnx2rtkp/gcc
make $1
if [ $? -ne 0 ]; then
    exit 3
fi
cd ../..

echo; echo % convbin/gcc
cd convbin/gcc
make $1
if [ $? -ne 0 ]; then
    exit 3
fi
cd ../..

echo; echo % rtkrcv/gcc
cd rtkrcv/gcc
make $1
if [ $? -ne 0 ]; then
    exit 3
fi
cd ../..

