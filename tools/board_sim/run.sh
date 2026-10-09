#!/bin/sh
# Builds and runs the simulated board. Extra -D flags go to the compiler;
# "--selftest" runs the scripted control-loop check instead.
set -e
cd "$(dirname "$0")"
FW=../..
CFLAGS="-std=gnu11 -O1 -Wall -Wno-unused-function"
ARGS=""
for a in "$@"; do
    case "$a" in
        -D*) CFLAGS="$CFLAGS $a" ;;
        *) ARGS="$ARGS $a" ;;
    esac
done
gcc $CFLAGS -I stub -I $FW/include -I $FW/src \
    board_sim.c \
    $FW/src/control/motor.c $FW/src/control/pid.c $FW/src/control/params.c \
    $FW/src/comms/protocol.c $FW/src/comms/can_proto.c $FW/src/comms/fmt.c \
    -lm -o /tmp/mdsim
exec /tmp/mdsim $ARGS
