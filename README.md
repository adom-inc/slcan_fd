# slcan_fd

A portable serial line CAN (slcan) interface for Rust with support for the newer CAN FD protocol.

Since sending and receiving CAN FD frames over an SLCAN interface is not standardized, this crate looks to be compatible with the commands supported by the [CANable 2.0 Firmware](https://github.com/normaldotcom/canable2-fw).

By default this crate is async and uses the [`tokio-serial`](https://github.com/berkowski/tokio-serial) crate, but it can also be used in a sync context (See [Cargo Features](#cargo-features)).

## Usage

```rust
use slcan_fd::{tokio::CanSocket, NominalBitRate, OperatingMode};
use tokio_serial::SerialPortBuilderExt;

let mut port = tokio_serial::new("/dev/ttyUSB0", 115_200).open_native_async()?;

#[cfg(unix)]
port.set_exclusive(false)
    .expect("Unable to set serial port exclusive to false");

let mut can = CanSocket::new(port);

can.close().await?;
can.set_operating_mode(OperatingMode::Silent).await?;
can.open(NominalBitRate::Rate500Kbit).await?;

loop {
    match can.read().await {
        Ok(frame) => println!("{:?}", frame),
        Err(e) => eprintln!("{:?}", e),
    }
}
```

## Cargo Features

The `tokio` feature is enabled by default.

- `tokio` - Implements the async API with the [`tokio-serial`](https://github.com/berkowski/tokio-serial) crate.
- `sync` - Implements the synchronous API with the [`serialport`](https://github.com/serialport/serialport-rs) crate.

## Credits

This crate is very loosely based on a previous crate for slcan without FD support which can be found [here](https://github.com/jmaygarden/slcan).
