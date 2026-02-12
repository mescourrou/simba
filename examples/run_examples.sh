#!/bin/bash

# Make the script stop if error occurs
set -e

echo "Run example 'basic'"
cd .. && cargo run --example basic && cd -
echo "Run example 'plugin_example'"
cd .. && cargo run -p plugin_example && cd -
echo "Run example 'python_external'"
cd .. && cargo run --example python_external && cd -

cd python
echo "Run python example 'controller'"
./controller.py
echo "Run python example 'navigator'"
./navigator.py
echo "Run python example 'physics'"
./physics.py
echo "Run python example 'state_estimator'"
./state_estimator.py
echo "Run python example 'messages'"
./messages.py

cd ..
echo "Run example 'stacked_plugins'"
cd stacked_plugins && ./run.sh