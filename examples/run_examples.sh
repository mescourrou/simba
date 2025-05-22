#!/bin/bash

cargo run -p basic

cd python
./controller.py
./navigator.py
./physics.py
./state_estimator.py