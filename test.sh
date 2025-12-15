#!/bin/bash
# Make the script stop if error occurs
set -e

cargo test
cargo build --all-features --release
cargo test --release --all-features

target/release/simba-cmd config_example/config_2.yaml --no-gui
target/release/simba-cmd config_example/config_scenario.yaml --no-gui

maturin develop

cd examples && ./run_examples.sh