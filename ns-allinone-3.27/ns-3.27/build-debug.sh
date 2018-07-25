#!/bin/bash

./waf configure --build-profile=debug --out=build/debug
./waf build
