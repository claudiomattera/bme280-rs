
set dotenv-load := true

# Print available recipes
default:
    @just --list

[private]
cargo +args:
    cargo {{args}}

# Fetch dependencies
fetch:
    @just cargo fetch --locked

# Check source code format
check-format: fetch
    @just cargo fmt --all -- --check

# Enforce source code format
format: fetch
    @just cargo fmt --all

# Type-check source code
check: fetch
    @just cargo check --frozen --all-targets --all-features

# Check lints with Clippy
lint: check
    @just cargo clippy --frozen --all-targets --all-features

# Build
build +args='--all-features': fetch
    @just cargo build --frozen {{args}}

# Build tests
build-tests +args='--all-features': fetch
    @just cargo test --frozen {{args}} --no-run

# Run tests
test +args='--all-features': (build-tests args)
    @just cargo test --frozen {{args}}

# Build documentation
build-documentation +args='--all-features': fetch
    @just cargo doc --frozen {{ args }}

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
