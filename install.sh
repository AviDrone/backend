#!/bin/sh
apt-get update              # To get the latest package lists
apt-get upgrade
apt install pipenv -y       # pipenv virtual environment
pipenv install
pip install -r requirements.txt
