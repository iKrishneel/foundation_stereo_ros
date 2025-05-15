#!/usr/bin/env bash

filename=model_best_bp2.pth

echo "Fetching files"
git lfs fetch --all origin master

echo "Merging files"
cat model_best_bp2.* > $filename
echo "Files merged to $filename"
