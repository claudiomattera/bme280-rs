
set dotenv-load := true

# Print available recipes
default:
    @just --list

[private]
cargo +args:
    cargo {{args}}

# Generate Cargo.lock
generate-lockfile:
    @just cargo generate-lockfile --offline

# Update Cargo.lock
update-lockfile:
    @just cargo update

# Fetch dependencies
fetch:
    @just cargo fetch

# Check source code format
check-format: fetch
    @just cargo fmt --all -- --check

# Enforce source code format
format: fetch
    @just cargo fmt --all

# Type-check source code
check +args='--all-features': fetch
    @just cargo check --frozen --all-targets {{args}}

# Type-check source code for all feature combinations
check-all-feature-combinations: fetch
    @just cargo hack --feature-powerset --no-dev-deps check

# Check lints with Clippy
lint +args='--all-features': (check args)
    @just cargo clippy --frozen --all-targets {{args}}

# Check lints with Clippy for all feature combinations
lint-all-feature-combinations: (check-all-feature-combinations)
    @just cargo hack --feature-powerset --no-dev-deps clippy

# Build debug
build +args='--all-features': fetch
    @just cargo build --frozen --all-targets {{args}}

# Build for all feature combinations
build-all-feature-combinations: (check-all-feature-combinations)
    @just cargo hack --feature-powerset --no-dev-deps build

# Build tests
build-tests +args='--all-features': fetch
    @just cargo test --frozen {{args}} --no-run

# Build tests for all feature combinations
build-tests-all-feature-combinations: (build-all-feature-combinations)
    @just cargo hack --feature-powerset test --no-run

# Run tests
test +args='--all-features': (build-tests args)
    @just cargo test --frozen {{args}}

# Run tests for all feature combinations
test-all-feature-combinations: (build-tests-all-feature-combinations)
    @just cargo hack --feature-powerset test

# Run an example
run-example *args: (build "--all-features")
    @just cargo run --frozen --all-features --example {{ args }}

# Build documentation
build-documentation +args='--all-features': fetch
    @just cargo doc --frozen --no-deps --document-private-items {{args}}

# Clean
clean:
    @just cargo clean

# Audit dependencies
audit:
    @just cargo audit --deny unsound --deny yanked

# Publish to crates.io
publish:
    @just cargo login "${CRATES_IO_TOKEN}"
    @just cargo publish
    @just cargo logout
