#!/usr/bin/env fish
virtualenv --python=python3 env
source env/bin/activate.fish
pip install --upgrade -r requirements.txt
pip install --upgrade -r requirements_dev.txt


# PySide dynamic library is not found without this
python env/bin/pyside_postinstall.py -install
