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

# Source the Espressif environment if the export script exists.
# This adds xtensa-esp32s3-elf-gcc and other ESP tools to PATH.
ESP_ENV := . $${HOME}/export-esp.sh 2>/dev/null;

HIL_DIR := hil

SIZE_DIR := size
SIZE_TARGET := xtensa-esp32s3-none-elf

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

.PHONY: help test test-all test-crc test-defmt test-minimal lint check check-hil check-size check-all doc \
        coverage hil-test hil-build size clean

# --------------------------------------------------------------------------- #
#	Library targets (no hardware)
# --------------------------------------------------------------------------- #

##@ Library (host — no hardware required)

test: ## Run unit tests (default features, ~185 tests)
	cargo test

test-all: ## Run unit tests with all features (~190 tests)
	cargo test --all-features

test-crc: ## Run unit tests with CRC feature (~190 tests)
	cargo test --features crc

test-defmt: ## Run unit tests with defmt feature
	cargo test --features defmt

test-minimal: ## Verify feature gates are additive (no default features)
	cargo test --no-default-features

lint: ## Run clippy on the library crate
	cargo clippy --all-features -- -D warnings

check: ## Verify driver compiles on ARM, RISC-V and Xtensa (no std/alloc)
	@printf "$(BOLD)Checking driver (thumbv7em-none-eabihf)...$(RESET)\n"
	cargo check --target thumbv7em-none-eabihf
	@printf "$(BOLD)Checking driver (riscv32imc-unknown-none-elf)...$(RESET)\n"
	cargo check --target riscv32imc-unknown-none-elf
	@printf "$(BOLD)Checking driver (xtensa-esp32s3-none-elf)...$(RESET)\n"
	@$(ESP_ENV) $(CARGO_ESP) check --target xtensa-esp32s3-none-elf -Zbuild-std=core

check-hil: ## Verify HIL crate compiles (requires esp toolchain)
	@$(ESP_ENV) ( cd $(HIL_DIR) && $(CARGO_ESP) check --release )

check-size: ## Verify size harness compiles (requires esp toolchain)
	@$(ESP_ENV) ( cd $(SIZE_DIR) && $(CARGO_ESP) check --release )

check-all: ## Run all compilation checks (driver + HIL + size)
	@$(MAKE) check
	@printf "$(BOLD)Checking HIL (xtensa-esp32s3-none-elf)...$(RESET)\n"
	@$(MAKE) check-hil
	@printf "$(BOLD)Checking size harness (xtensa-esp32s3-none-elf)...$(RESET)\n"
	@$(MAKE) check-size
	@printf "\n$(BOLD)All checks passed.$(RESET)\n"

doc: ## Build docs locally with feature annotations
	cargo doc --all-features --open

coverage: ## Generate test coverage report (lcov + HTML)
	@mkdir -p coverage
	cargo llvm-cov --all-features --lcov --output-path coverage/lcov.info
	cargo llvm-cov --all-features --html --output-dir coverage/html
	@printf "\n$(BOLD)Coverage report:$(RESET)\n"
	@printf "  lcov:  coverage/lcov.info\n"
	@printf "  HTML:  coverage/html/html/index.html\n\n"

size: ## Report .text size per feature combo (requires esp toolchain)
	@$(ESP_ENV) \
	LLVM_SIZE=$$(find "$$($(CARGO_ESP) rustc --print sysroot 2>/dev/null || rustc --print sysroot)" -name llvm-size 2>/dev/null | head -1); \
	if [ -z "$$LLVM_SIZE" ]; then \
		printf "$(BOLD)Error:$(RESET) llvm-size not found. Install with: rustup component add llvm-tools\n"; \
		exit 1; \
	fi; \
	printf "\n$(BOLD)tmag5273 binary size ($(SIZE_TARGET), release)$(RESET)\n\n"; \
	printf "  $(CYAN)%-20s$(RESET) %8s %8s %8s\n" "Features" ".text" ".data" ".bss"; \
	printf "  %-20s %8s %8s %8s\n" "--------" "-----" "-----" "----"; \
	for combo in "default:--no-default-features" "crc:--features crc" "defmt:--features defmt" "crc+defmt:--features crc,defmt"; do \
		name=$${combo%%:*}; flags=$${combo#*:}; \
		if ! ( cd $(SIZE_DIR) && RUSTFLAGS="-C llvm-args=--disable-block-placement -C llvm-args=--disable-branch-fold" $(CARGO_ESP) build --release $$flags 2>/dev/null ); then \
			printf "  %-20s %8s %8s %8s\n" "$$name" "FAILED" "-" "-"; \
			continue; \
		fi; \
		sizes=$$($$LLVM_SIZE $(SIZE_DIR)/target/$(SIZE_TARGET)/release/tmag5273-size 2>/dev/null | tail -1); \
		text=$$(echo "$$sizes" | awk '{print $$1}'); \
		data=$$(echo "$$sizes" | awk '{print $$2}'); \
		bss=$$(echo "$$sizes" | awk '{print $$3}'); \
		printf "  %-20s %8s %8s %8s\n" "$$name" "$$text" "$$data" "$$bss"; \
	done; \
	printf "\n"

# --------------------------------------------------------------------------- #
#	HIL targets (requires ESP32-S3 + TMAG5273 hardware)
# --------------------------------------------------------------------------- #

##@ HIL (hardware-in-the-loop — requires ESP32-S3 + TMAG5273)

hil-test: ## Flash and run all HIL tests via probe-rs
	@$(ESP_ENV) ( cd $(HIL_DIR) && $(CARGO_ESP) test --release )

hil-build: ## Build HIL test binary without flashing
	@$(ESP_ENV) ( cd $(HIL_DIR) && $(CARGO_ESP) build --release )

# --------------------------------------------------------------------------- #
#	Housekeeping
# --------------------------------------------------------------------------- #

##@ Housekeeping

clean: ## Remove all build artifacts
	cargo clean && cargo update
	( cd $(HIL_DIR) && cargo clean && cargo update)
	( cd $(SIZE_DIR) && cargo clean && cargo update 2>/dev/null || true )

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