#!/bin/bash

set -e

TRACE=$1
MODEL=${TRACE/traces/problems}
MODEL_SRC=${MODEL/.nbt/_src.mdl}
MODEL_DST=${MODEL/.nbt/_tgt.mdl}

(rust/scorer/target/release/scorer -t $TRACE -s $MODEL_SRC -d $MODEL_DST | grep 'FINAL ENERGY' | cut -f4 -d ' ') \
    | read ORIG_SCORE
ORIG_SCORE=$(($ORIG_SCORE+0))

NEW_TRACE=`mktemp`

rust/random_swarm/target/release/random_swarm \
    -s $MODEL_SRC -t $MODEL_DST -o $NEW_TRACE \
    --global-ticks-limit 131072 --rtt-limit 256 --route-attempts-limit 512

(rust/scorer/target/release/scorer -t $NEW_TRACE -s $MODEL_SRC -d $MODEL_DST | grep 'FINAL ENERGY' | cut -f4 -d ' ') \
    | read NEW_SCORE
NEW_SCORE=$(($NEW_SCORE+0))

if [ ${NEW_SCORE} -lt ${ORIG_SCORE} ]
then
    echo "IMPROVED $TRACE: was $ORIG_SCORE now $NEW_SCORE"
    mv -f $NEW_TRACE $TRACE
fi
rm -f $NEW_TRACE
