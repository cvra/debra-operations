#!/usr/bin/env fish
virtualenv --python=python3 env
source env/bin/activate.fish 
pip install -r requirements.txt
