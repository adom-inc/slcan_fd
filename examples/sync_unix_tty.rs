use serialport::TTYPort;
use slcan_fd::{sync::CanSocket, NominalBitRate, ReadError};

fn main() -> std::io::Result<()> {
    let arg = std::env::args().nth(1);

    let Some(filename) = arg else {
        eprintln!("usage: unix_tty <TTY path>");
        std::process::exit(1);
    };

    let port = serialport::TTYPort::open(&serialport::new(filename, 115200))?;

    let mut can = CanSocket::<TTYPort>::new(port);

    can.close()?;
    can.set_operating_mode(slcan_fd::OperatingMode::Silent)?;
    can.open(NominalBitRate::Rate500Kbit)?;

    loop {
        match can.read() {
            Ok(frame) => println!("{:?}", frame),
            Err(ReadError::Io(e))
                if matches!(
                    e.kind(),
                    std::io::ErrorKind::TimedOut | std::io::ErrorKind::WouldBlock
                ) => {}
            e => eprintln!("{:?}", e),
        }
    }
}
