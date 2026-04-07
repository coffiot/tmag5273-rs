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
MONITOR_DIR := monitor

SIZE_DIR := size
SIZE_TARGET := xtensa-esp32s3-none-elf

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

.PHONY: help test test-all test-crc test-defmt test-minimal lint check check-hil check-monitor check-size check-all \
        build build-hil build-monitor build-all doc coverage verify \
        hil-test-non-interactive hil-test-idle-interactive hil-test-spinning-interactive \
        hil-test-single-magnet hil-test-interactive hil-build monitor size clean

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

check-hil: ## Verify HIL crate compiles (all feature combos)
	@printf "$(BOLD)Checking HIL (default)...$(RESET)\n"
	@$(ESP_ENV) ( cd $(HIL_DIR) && $(CARGO_ESP) check --release )
	@printf "$(BOLD)Checking HIL (interactive)...$(RESET)\n"
	@$(ESP_ENV) ( cd $(HIL_DIR) && $(CARGO_ESP) check --release --features interactive )
	@printf "$(BOLD)Checking HIL (single-magnet)...$(RESET)\n"
	@$(ESP_ENV) ( cd $(HIL_DIR) && $(CARGO_ESP) check --release --features single-magnet )

check-monitor: ## Verify monitor crate compiles
	@printf "$(BOLD)Checking monitor...$(RESET)\n"
	@$(ESP_ENV) ( cd $(MONITOR_DIR) && $(CARGO_ESP) check --release )

check-size: ## Verify size harness compiles
	@printf "$(BOLD)Checking size harness...$(RESET)\n"
	@$(ESP_ENV) ( cd $(SIZE_DIR) && $(CARGO_ESP) check --release )

check-all: check check-hil check-monitor check-size ## All compilation checks (driver + HIL + monitor + size)
	@printf "\n$(BOLD)$(GREEN)All compilation checks passed.$(RESET)\n"

# --------------------------------------------------------------------------- #
#	Build targets
# --------------------------------------------------------------------------- #

##@ Build

build: ## Build driver (all features, host target)
	cargo build --all-features

build-hil: ## Build all HIL test binaries without flashing
	@$(ESP_ENV) ( cd $(HIL_DIR) && $(CARGO_ESP) build --release )
	@$(ESP_ENV) ( cd $(HIL_DIR) && $(CARGO_ESP) build --release --features interactive )
	@$(ESP_ENV) ( cd $(HIL_DIR) && $(CARGO_ESP) build --release --features single-magnet )

build-monitor: ## Build monitor binary without flashing
	@$(ESP_ENV) ( cd $(MONITOR_DIR) && $(CARGO_ESP) build --release )

build-all: build build-hil build-monitor ## Build everything (driver + HIL + monitor)
	@printf "\n$(BOLD)$(GREEN)All builds succeeded.$(RESET)\n"

# --------------------------------------------------------------------------- #
#	Unified verify target
# --------------------------------------------------------------------------- #

##@ Quality

verify: test lint check-all ## Run tests + lint + all compilation checks
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

hil-test-non-interactive: ## Non-interactive: flash and run HIL tests (no magnet needed)
	@$(ESP_ENV) ( cd $(HIL_DIR) && $(CARGO_ESP) test --release --bin tmag5273-non-interactive )

hil-test-idle-interactive: ## Interactive: hold Gicar frother still, motor OFF
	@$(ESP_ENV) ( cd $(HIL_DIR) && $(CARGO_ESP) test --release --features interactive --bin tmag5273-interactive-idle )

hil-test-spinning-interactive: ## Interactive: spin Gicar frother continuously
	@$(ESP_ENV) ( cd $(HIL_DIR) && $(CARGO_ESP) test --release --features interactive --bin tmag5273-interactive-spinning )

hil-test-single-magnet: ## Interactive: hold single bar magnet near sensor (CORDIC tests)
	@$(ESP_ENV) ( cd $(HIL_DIR) && $(CARGO_ESP) test --release --features single-magnet --bin tmag5273-interactive-single-magnet )

hil-test-interactive: ## Interactive: run idle + spinning sequentially (prompts between)
	@printf "$(BOLD)$(CYAN)Step 1/2: IDLE tests$(RESET) — hold Gicar frother near sensor, motor OFF\n"
	@$(MAKE) hil-test-idle-interactive
	@printf "\n$(BOLD)$(MAGENTA)Idle tests passed.$(RESET)\n"
	@printf "$(BOLD)$(CYAN)Step 2/2: SPINNING tests$(RESET) — start frother now and keep spinning.\n"
	@read -p "Press Enter when frother is spinning... " _
	@$(MAKE) hil-test-spinning-interactive
	@printf "\n$(BOLD)All interactive tests passed.$(RESET)\n"

monitor: ## Flash and run live magnet monitor demo (Ctrl+C to stop)
	@$(ESP_ENV) ( cd $(MONITOR_DIR) && $(CARGO_ESP) run --release )

# --------------------------------------------------------------------------- #
#	Housekeeping
# --------------------------------------------------------------------------- #

##@ Housekeeping

clean: ## Remove all build artifacts
	cargo clean
	( cd $(HIL_DIR) && cargo clean )
	( cd $(MONITOR_DIR) && cargo clean 2>/dev/null || true )
	( cd $(SIZE_DIR) && cargo clean 2>/dev/null || true )

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