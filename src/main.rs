#![feature(never_type)]
use std::path::Path;
use std::sync::OnceLock;

use clap::Parser;

use evdev::uinput::VirtualDevice;
use evdev::{
    AttributeSet, AttributeSetRef, Device, EventType, InputEvent, KeyCode, RelativeAxisCode,
};

const VIRTUAL_DEVICE_NAME: &str = "sensor alignment virtual device";

#[derive(Parser, Debug)]
#[command(version, about, long_about = None)]
struct Args {
    #[arg(short, long)]
    device_path: String,
    #[arg(short, long)]
    angle_deg: f64,
}

fn main() -> std::io::Result<()> {
    let args = Args::parse();

    let (cos_a, sin_a) = {
        let angle_rad = args.angle_deg.to_radians();
        (angle_rad.cos(), angle_rad.sin())
    };

    loop {
        let mut input_device = match with_retry(
            || create_input_device(&args.device_path),
            "Creating input device",
            10,
        ) {
            Ok(device) => device,
            Err(_) => continue,
        };

        let relative_axes = input_device
            .supported_relative_axes()
            .unwrap_or(default_rel_axes());

        let keys = input_device.supported_keys().unwrap_or(default_keys());

        let mut virtual_device = match with_retry(
            || create_virtual_device(VIRTUAL_DEVICE_NAME, relative_axes, keys),
            "Creating virtual device",
            10,
        ) {
            Ok(device) => device,
            Err(_) => continue,
        };

        println!("\nconfig:");
        println!("  angle: {}°", args.angle_deg);
        if let Some(name) = input_device.name() {
            println!("  device: {} ({})", name, args.device_path);
        } else {
            println!("  device: Unknown ({})", args.device_path);
        };
        println!(
            "  virtual device: {} ({})\n",
            VIRTUAL_DEVICE_NAME,
            virtual_device
                .get_syspath()
                .map(|path| path.to_string_lossy().into_owned())
                .unwrap_or("Unknown".to_string())
        );

        input_device.grab()?;

        match with_retry(
            || {
                let res = event_loop(&mut input_device, &mut virtual_device, sin_a, cos_a);
                let _ = input_device.ungrab();
                res
            },
            "Event loop",
            10,
        ) {
            Ok(_) => unreachable!(),
            Err(_) => {}
        };
    }
}

fn with_retry<T, F>(mut action: F, name: &str, seconds: u64) -> std::io::Result<T>
where
    F: FnMut() -> std::io::Result<T>,
{
    loop {
        match action() {
            Ok(v) => return Ok(v),
            Err(e) => {
                eprintln!("{name} failed: {e}. Retrying in {seconds} seconds...");
                std::thread::sleep(std::time::Duration::from_secs(seconds));
            }
        }
    }
}

fn default_rel_axes() -> &'static AttributeSet<RelativeAxisCode> {
    static DEFAULT: OnceLock<AttributeSet<RelativeAxisCode>> = OnceLock::new();
    DEFAULT.get_or_init(|| {
        let mut set = AttributeSet::new();
        set.insert(RelativeAxisCode::REL_X);
        set.insert(RelativeAxisCode::REL_Y);
        set.insert(RelativeAxisCode::REL_WHEEL);
        set
    })
}

fn default_keys() -> &'static AttributeSet<KeyCode> {
    static DEFAULT: OnceLock<AttributeSet<KeyCode>> = OnceLock::new();
    DEFAULT.get_or_init(|| {
        let mut set = AttributeSet::new();
        set.insert(KeyCode::BTN_LEFT);
        set.insert(KeyCode::BTN_RIGHT);
        set.insert(KeyCode::BTN_MIDDLE);
        set
    })
}

fn create_input_device(path: impl AsRef<Path>) -> std::io::Result<Device> {
    let mut input_device = Device::open(path)?;
    input_device.grab()?;
    Ok(input_device)
}

fn create_virtual_device(
    name: &str,
    relative_axes: &AttributeSetRef<RelativeAxisCode>,
    keys: &AttributeSetRef<KeyCode>,
) -> std::io::Result<VirtualDevice> {
    let virtual_device = VirtualDevice::builder()?
        .name(name)
        .with_relative_axes(relative_axes)?
        .with_keys(keys)?
        .build()?;
    Ok(virtual_device)
}

fn event_loop(
    input_device: &mut Device,
    virtual_device: &mut VirtualDevice,
    sin_a: f64,
    cos_a: f64,
) -> std::io::Result<!> {
    let mut dx: i32 = 0;
    let mut dy: i32 = 0;

    loop {
        for event in input_device.fetch_events()? {
            match event.event_type() {
                EventType::RELATIVE => {
                    let relative_axis_code = RelativeAxisCode(event.code());

                    match relative_axis_code {
                        RelativeAxisCode::REL_X => dx += event.value(),
                        RelativeAxisCode::REL_Y => dy += event.value(),
                        _ => virtual_device.emit(&[event])?,
                    }
                }
                EventType::SYNCHRONIZATION => {
                    if dx != 0 || dy != 0 {
                        let new_dx = (dx as f64 * cos_a - dy as f64 * sin_a).round() as i32;
                        let new_dy = (dx as f64 * sin_a + dy as f64 * cos_a).round() as i32;

                        virtual_device.emit(&[
                            InputEvent::new_now(
                                EventType::RELATIVE.0,
                                RelativeAxisCode::REL_X.0,
                                new_dx,
                            ),
                            InputEvent::new_now(
                                EventType::RELATIVE.0,
                                RelativeAxisCode::REL_Y.0,
                                new_dy,
                            ),
                            event,
                        ])?;

                        dx = 0;
                        dy = 0;
                    } else {
                        virtual_device.emit(&[event])?;
                    }
                }
                _ => virtual_device.emit(&[event])?,
            }
        }
    }
}
