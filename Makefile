# tmag5273 — Build, test, and quality targets
#
# Library targets run on the host (no hardware needed).
# Field-test crates (HIL, monitor, size harness) live in a sibling repo:
# https://github.com/coffiot/tmag5273-field-tests

.DEFAULT_GOAL := help

# --------------------------------------------------------------------------- #
#	Config
# --------------------------------------------------------------------------- #

# Espressif Rust toolchain — only used by the cross-compile check below.
CARGO_ESP ?= cargo +esp

# Source the Espressif environment if the export script exists.
# This adds xtensa-esp32s3-elf-gcc and other ESP tools to PATH.
ESP_ENV := . $${HOME}/export-esp.sh 2>/dev/null;

# --------------------------------------------------------------------------- #
#	ANSI formatting
# --------------------------------------------------------------------------- #

BOLD := \033[1m
CYAN := \033[36m
GREEN := \033[32m
MAGENTA := \033[35m
RESET := \033[0m

# --------------------------------------------------------------------------- #
#	Phony declarations
# --------------------------------------------------------------------------- #

.PHONY: help test test-all test-crc test-defmt test-minimal lint check \
        build doc coverage verify clean

# --------------------------------------------------------------------------- #
#	Library targets (no hardware)
# --------------------------------------------------------------------------- #

##@ Library (host — no hardware required)

test: ## Run unit tests (~370, all features)
	cargo test --all-features

test-all: test ## Alias for test (all features)

test-crc: ## Run unit tests with CRC feature only
	cargo test --features crc

test-defmt: ## Run unit tests with defmt feature only
	cargo test --features defmt

test-minimal: ## Verify feature gates are additive (no default features)
	cargo test --no-default-features

lint: ## Run clippy on the library crate
	cargo clippy --all-features -- -D warnings

# --------------------------------------------------------------------------- #
#	Compilation checks (no hardware)
# --------------------------------------------------------------------------- #

##@ Compilation checks (no hardware)

check: ## Verify driver compiles on ARM, RISC-V and Xtensa (no std/alloc)
	@printf "$(BOLD)Checking driver (thumbv7em-none-eabihf)...$(RESET)\n"
	cargo check --target thumbv7em-none-eabihf
	@printf "$(BOLD)Checking driver (riscv32imc-unknown-none-elf)...$(RESET)\n"
	cargo check --target riscv32imc-unknown-none-elf
	@printf "$(BOLD)Checking driver (xtensa-esp32s3-none-elf)...$(RESET)\n"
	@$(ESP_ENV) $(CARGO_ESP) check --target xtensa-esp32s3-none-elf -Zbuild-std=core

# --------------------------------------------------------------------------- #
#	Build targets
# --------------------------------------------------------------------------- #

##@ Build

build: ## Build driver (all features, host target)
	cargo build --all-features

# --------------------------------------------------------------------------- #
#	Unified verify target
# --------------------------------------------------------------------------- #

##@ Quality

verify: test lint check ## Run tests + lint + cross-compile checks
	@printf "\n$(BOLD)$(GREEN)════════════════════════════════════════$(RESET)\n"
	@printf "$(BOLD)$(GREEN)  All verification passed.$(RESET)\n"
	@printf "$(BOLD)$(GREEN)════════════════════════════════════════$(RESET)\n\n"

doc: ## Build docs locally with feature annotations
	cargo doc --all-features --open

coverage: ## Generate test coverage report (lcov + HTML)
	@mkdir -p coverage
	cargo llvm-cov --all-features --lcov --output-path coverage/lcov.info
	cargo llvm-cov --all-features --html --output-dir coverage/html
	@printf "\n$(BOLD)Coverage report:$(RESET)\n"
	@printf "  lcov:  coverage/lcov.info\n"
	@printf "  HTML:  coverage/html/html/index.html\n\n"

# --------------------------------------------------------------------------- #
#	Housekeeping
# --------------------------------------------------------------------------- #

##@ Housekeeping

clean: ## Remove all build artifacts
	cargo clean

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
		printf "  $(CYAN)%-30s$(RESET) %s\n", $$1, $$2 \
	}' $(MAKEFILE_LIST)
	@printf "\n"
