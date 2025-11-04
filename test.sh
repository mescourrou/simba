#!/bin/bash
# Make the script stop if error occurs
set -e

cargo test --all-features
cargo test

maturin develop

cd examples && ./run_examples.sh