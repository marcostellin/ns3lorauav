#!/bin/bash

./waf configure --build-profile=optimized --out=build/optimized
./waf build
