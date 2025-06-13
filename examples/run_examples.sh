#!/bin/bash

cd .. && cargo run --example basic && cd -

cd python
./controller.py
./navigator.py
./physics.py
./state_estimator.py