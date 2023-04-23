# Pico networking

HAL based networking examples for the Raspberry Pi Pico, Pico-W, RP2040 using Rust.

## Installation of development dependencies

```shell
rustup target install thumbv6m-none-eabi
cargo install flip-link
# This is our suggested default 'runner'
cargo install probe-rs-cli
# If you want to use elf2uf2-rs instead of probe-run, instead do...
cargo install elf2uf2-rs
```

## Running

For a debug build

```shell
cargo run
```

For a release build

```shell
cargo run --release
```

If you do not specify a DEFMT_LOG level, it will be set to `debug`.
That means `println!("")`, `info!("")` and `debug!("")` statements will be printed.
If you wish to override this, you can change it in `.cargo/config.toml`

```toml
[env]
DEFMT_LOG = "off"

```

You can also set this inline (on Linux/MacOS)

```sh
DEFMT_LOG=trace cargo run
```

or set the _environment variable_ so that it applies to every `cargo run` call that follows:

### Linux/MacOS/unix

```sh
export DEFMT_LOG=trace
```

Setting the DEFMT_LOG level for the current session  
for bash

```sh
export DEFMT_LOG=trace
```

### Windows

Windows users can only override DEFMT_LOG through `config.toml`
or by setting the environment variable as a separate step before calling `cargo run`

- cmd

```cmd
set DEFMT_LOG=trace
```

- powershell

```ps1
$Env:DEFMT_LOG = trace
```

```cmd
cargo run
```
