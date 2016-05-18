#!/bin/sh
. ./env/bin/activate
mkdir -p ipc
python launcher.py launcher.json
