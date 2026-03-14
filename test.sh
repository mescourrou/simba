#!/bin/bash
# Make the script stop if error occurs
set -e

cargo nextest run --no-default-features
cargo build --release
cargo nextest run --release
cargo nextest run --release --all-features

target/release/simba-cmd config_example/config_2.yaml --no-gui
target/release/simba-cmd config_example/config_scenario.yaml --no-gui

maturin develop

cd examples && ./run_examples.sh