#!/bin/bash

set -e

TRACE=$1
MODEL=${TRACE/traces/problems}
MODEL=${MODEL/.nbt/_tgt.mdl}

ORIG_SCORE=$(rust/scorer/target/release/scorer -t $TRACE -d $MODEL | grep ENERGY | cut -f2 -d ' ')
ORIG_SCORE=$(($ORIG_SCORE+0))

NEW_TRACE=`mktemp`

rust/random_swarm/target/release/random_swarm \
    -t $MODEL -o $NEW_TRACE \
    --global-ticks-limit 131072 --rtt-limit 256 --route-attempts-limit 512

NEW_SCORE=$(rust/scorer/target/release/scorer -t $NEW_TRACE -d $MODEL | grep ENERGY | cut -f2 -d ' ')
NEW_SCORE=$(($NEW_SCORE+0))

if [[ $NEW_SCORE -lt $OLD_SCORE ]];
then
    echo "IMPROVED $TRACE: was $ORIG_SCORE now $NEW_SCORE"
    mv -f $NEW_TRACE $TRACE
fi
