#!/bin/bash

set -e

TRACE=$1
MODEL=${TRACE/traces/problems}
MODEL=${MODEL/.nbt/_tgt.mdl}

(rust/scorer/target/release/scorer -t $TRACE -d $MODEL | grep 'FINAL ENERGY' | cut -f4 -d ' ') \
    | read ORIG_SCORE
ORIG_SCORE=$(($ORIG_SCORE+0))

NEW_TRACE=`mktemp`

rust/trace-gravitizer/target/release/trace-gravitizer \
    -t $TRACE -f $NEW_TRACE

(rust/scorer/target/release/scorer -t $NEW_TRACE -d $MODEL | grep 'FINAL ENERGY' | cut -f4 -d ' ') \
    | read NEW_SCORE
NEW_SCORE=$(($NEW_SCORE+0))

if [[ $NEW_SCORE -lt $OLD_SCORE ]];
then
    echo "IMPROVED $TRACE: was $ORIG_SCORE now $NEW_SCORE"
    mv -f $NEW_TRACE $TRACE
fi
