#!/bin/sh
IP=${IP:='172.30.147.56'}
make -j4 && scp bin/kernel.img pi@${IP}:~/ && ssh pi@${IP} ./runKernel.sh
