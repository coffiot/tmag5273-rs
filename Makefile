# tmag5273 — Build, test, and HIL targets
#
# Library targets run on the host (no hardware needed).
# HIL targets require: espup, probe-rs-tools, ESP32-S3 via USB JTAG,
# TMAG5273 B1 wired to GPIO8 (SDA) / GPIO9 (SCL).

.DEFAULT_GOAL := help

# --------------------------------------------------------------------------- #
#	Config
# --------------------------------------------------------------------------- #

# Espressif Rust toolchain (override: CARGO_ESP="cargo +nightly" make hil-test)
CARGO_ESP ?= cargo +esp

HIL_DIR := hil

# --------------------------------------------------------------------------- #
#	ANSI formatting
# --------------------------------------------------------------------------- #

BOLD := \033[1m
CYAN := \033[36m
MAGENTA := \033[35m
RESET := \033[0m

# --------------------------------------------------------------------------- #
#	Phony declarations
# --------------------------------------------------------------------------- #

.PHONY: help test test-all test-crc test-defmt lint check doc \
        hil-test hil-build clean

# --------------------------------------------------------------------------- #
#	Library targets (no hardware)
# --------------------------------------------------------------------------- #

##@ Library (host — no hardware required)

test: ## Run unit tests (default features, ~185 tests)
	cargo test -p tmag5273

test-all: ## Run unit tests with all features (~190 tests)
	cargo test -p tmag5273 --all-features

test-crc: ## Run unit tests with CRC feature (~190 tests)
	cargo test -p tmag5273 --features crc

test-defmt: ## Verify feature gates are additive (no default features)
	cargo test -p tmag5273 --no-default-features

lint: ## Run clippy on the library crate
	cargo clippy -p tmag5273 --all-features -- -D warnings

check: ## Verify bare-metal compilation (no std/alloc)
	cargo check -p tmag5273 --target thumbv7em-none-eabihf

doc: ## Build docs locally with feature annotations
	cargo doc -p tmag5273 --all-features --open

# --------------------------------------------------------------------------- #
#	HIL targets (requires ESP32-S3 + TMAG5273 hardware)
# --------------------------------------------------------------------------- #

##@ HIL (hardware-in-the-loop — requires ESP32-S3 + TMAG5273)

hil-test: ## Flash and run all HIL tests via probe-rs (7 tests)
	( cd $(HIL_DIR) && $(CARGO_ESP) test --release )

hil-build: ## Build HIL test binary without flashing
	( cd $(HIL_DIR) && $(CARGO_ESP) build --release )

# --------------------------------------------------------------------------- #
#	Housekeeping
# --------------------------------------------------------------------------- #

##@ Housekeeping

clean: ## Remove all build artifacts
	cargo clean
	( cd $(HIL_DIR) && cargo clean )

# --------------------------------------------------------------------------- #
#	Help
# --------------------------------------------------------------------------- #

help: ## Show this help message
	@awk 'BEGIN { \
		FS = ":.*##"; \
		printf "\n$(BOLD)tmag5273 Build System$(RESET)\n\n"; \
		printf "$(CYAN)Usage:$(RESET) make <target>\n\n" \
	} \
	/^##@/ { \
		if (section != "") printf "\n"; \
		section = substr($$0, 5); \
		printf "$(BOLD)$(MAGENTA)%s$(RESET)\n", section \
	} \
	/^[a-zA-Z][a-zA-Z0-9_-]*:.*##/ { \
		printf "  $(CYAN)%-20s$(RESET) %s\n", $$1, $$2 \
	}' $(MAKEFILE_LIST)
	@printf "\n"