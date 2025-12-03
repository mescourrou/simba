#!/bin/env bash

set -e

CURRENT_RELEASE=$(git branch --show-current | sed 's/release\/v//')

echo "Preparing release $CURRENT_RELEASE"

if [ -z "$CURRENT_RELEASE" ]; then
    echo "Could not determine current release from git branch"
    exit 1
fi

CARGO_RELEASE_VERSION=$(cat Cargo.toml | grep '^version =' | head -n 1 | sed 's/version = "//' | sed 's/"//')
if [ "$CARGO_RELEASE_VERSION" != "$CURRENT_RELEASE" ]; then
    echo "Cargo version ($CARGO_RELEASE_VERSION) does not match release version ($CURRENT_RELEASE)"
    exit 1
fi

CHANGELOG_VERSION_LINE=$(cat CHANGELOGS.md | grep "$CURRENT_RELEASE" | head -n 1)
if [ -z "$CHANGELOG_VERSION_LINE" ]; then
    echo "Could not find changelog entry for version $CURRENT_RELEASE"
    exit 1
fi

cargo fmt --all
cargo clippy --all-features --all-targets -- -D warnings
cargo build --release --all-features
cargo doc --no-deps --all-features --release
./generate_config_doc.py
target/release/simba-tools --generate-schema config.schema.json

./test.sh