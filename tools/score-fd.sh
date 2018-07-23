#!/bin/bash

set -e

TRACE=$1
MODEL=${TRACE/traces/problems}
MODEL=${MODEL/.nbt/_src.mdl}

echo "$TRACE: Start processing"
ORIG_SCORE=$(rust/scorer/target/release/scorer -t $TRACE -s $MODEL | grep 'FINAL ENERGY' | cut -f4 -d ' ') \
if [ "$ORIG_SCORE" == "" ]; then
    echo "$TRACE: unable to score orig trace"
    exit 1
fi
ORIG_SCORE=$(($ORIG_SCORE+0))
echo "$TRACE: Orig score: $ORIG_SCORE"

NEW_TRACE=`mktemp`

rust/random_swarm/target/release/random_swarm \
    -s $MODEL -o $NEW_TRACE \
    -M 12 \
    --global-ticks-limit 131072 --rtt-limit 256 --route-attempts-limit 512

NEW_SCORE=$(rust/scorer/target/release/scorer -t $NEW_TRACE -s $MODEL | grep 'FINAL ENERGY' | cut -f4 -d ' ')
if [ "$NEW_SCORE" == "" ]; then
    echo "$TRACE: unable to score the new trace"
    echo "$TRACE: check $NEW_TRACE for debug"
    exit 1
fi
NEW_SCORE=$(($NEW_SCORE+0))

echo "$TRACE: was $ORIG_SCORE now $NEW_SCORE"

if [ ${NEW_SCORE} -lt ${ORIG_SCORE} ]
then
    echo "IMPROVED $TRACE: was $ORIG_SCORE now $NEW_SCORE"
    mv -f $NEW_TRACE $TRACE
fi
rm -f $NEW_TRACE
