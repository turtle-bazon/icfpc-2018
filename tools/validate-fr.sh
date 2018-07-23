#!/bin/bash

set -e

TRACE=$1
MODEL=${TRACE/traces/problems}
MODEL_SRC=${MODEL/.nbt/_src.mdl}
MODEL_DST=${MODEL/.nbt/_tgt.mdl}

rust/scorer/target/release/scorer -t $TRACE -s $MODEL_SRC -d $MODEL_DST
