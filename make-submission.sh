#!/bin/bash

set -e

SUBMISSION_NAME=$1
SUBMISSION_PASS=$2

# Cleanup
rm -rf .submission/ $SUBMISSION_NAME.zip $SUBMISSION_NAME.zip.sha256sum

mkdir .submission

git archive --format=tar HEAD | tar x -C .submission/

cd .submission
zip -9 -e -P $SUBMISSION_PASS -r ../$SUBMISSION_NAME.zip .
cd ..

shasum -a 256 $SUBMISSION_NAME.zip > $SUBMISSION_NAME.zip.hash
