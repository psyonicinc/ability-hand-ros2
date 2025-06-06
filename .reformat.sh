#!/usr/bin/env bash

rm -rf build install log

find . -name '*.py' \
    | xargs black --safe --line-length=79 --target-version=py312
