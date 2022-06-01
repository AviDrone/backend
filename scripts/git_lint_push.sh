#!/bin/bash
black .
isort .

git add .
git commit -a -m "Linted using black and isort"
git push