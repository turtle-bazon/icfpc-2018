#!/bin/bash

set -e

TRACE=$1
MODEL=${TRACE/traces/problems}
MODEL=${MODEL/.nbt/_src.mdl}

rust/scorer/target/release/scorer -t $TRACE -s $MODEL
