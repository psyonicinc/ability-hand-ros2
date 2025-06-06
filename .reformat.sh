#!/usr/bin/env bash

find . -name '*.py' \
    | xargs black --safe --line-length=79 --target-version=py312
