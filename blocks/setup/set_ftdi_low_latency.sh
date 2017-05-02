#!/bin/sh

OLDPATH=/sys${DEVPATH}/../../latency_timer
NEWPATH=/sys${DEVPATH}/latency_timer
if [ -e "${OLDPATH}" ]; then
    echo 1 > ${OLDPATH}
    logger "Setting latency timer of ${DEVPATH} to 1"
elif [ -e "${NEWPATH}" ]; then
    echo 1 > ${NEWPATH}
    logger "Setting latency timer of ${DEVPATH} to 1"
fi
