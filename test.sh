#!/bin/bash
# Make the script stop if error occurs
set -e

cargo test --all-features
cargo test

cargo build --all-features --release
target/release/simba-cmd config_example/config_2.yaml --no-gui

maturin develop

cd examples && ./run_examples.sh