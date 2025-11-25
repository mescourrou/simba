#!/bin/bash

# Make the script stop if error occurs
set -e

maturin develop
./python_plugin.py