#!/bin/bash
# Make the script stop if error occurs
set -e

cargo test --all-features
cargo test

cd examples && ./run_examples.sh