#!/bin/bash

# Make the script stop if error occurs
set -e

cd .. && cargo run --example basic && cd -
cd .. && cargo run --example plugin && cd -
cd .. && cargo run --example python_external && cd -

cd python
./controller.py
./navigator.py
./physics.py
./state_estimator.py
./messages.py

cd ..
cd stacked_plugins && ./run.sh