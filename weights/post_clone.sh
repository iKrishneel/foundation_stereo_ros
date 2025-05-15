#!/usr/bin/env bash

filename=$HOME/.cache/torch/model_best_bp2.pth
dir_path="$(dirname "$filename")"
mkdir -p $dir_path

echo -e "\033[32mFetching files\033[0m"
git lfs fetch --all origin master

echo -e "\033[32mMerging files\033[0m"
cat model_best_bp2.a* > $filename
echo -e "\033[36mFiles merged and saved to >>> $filename\033[0m"
