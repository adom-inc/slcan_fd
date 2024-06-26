use slcan_fd::{tokio::CanSocket, NominalBitRate, OperatingMode};
use tokio_serial::SerialPortBuilderExt;

#[tokio::main]
async fn main() -> std::io::Result<()> {
    let arg = std::env::args().nth(1);

    let Some(filename) = arg else {
        eprintln!("usage: unix_tty <TTY path>");
        std::process::exit(1);
    };

    let mut port = tokio_serial::new(filename, 115_200).open_native_async()?;

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
}
