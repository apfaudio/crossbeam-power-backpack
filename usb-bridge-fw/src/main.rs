//! # Pico USB Serial (with Interrupts) Example
//!
//! Creates a USB Serial device on a Pico board, with the USB driver running in
//! the USB interrupt.
//!
//! This will create a USB Serial device echoing anything it receives. Incoming
//! ASCII characters are converted to upercase, so you can tell it is working
//! and not just local-echo!
//!
//! See the `Cargo.toml` file for Copyright and license details.

#![no_std]
#![no_main]

// The macro for our start-up function
use rp_pico::entry;

// The macro for marking our interrupt functions
use rp_pico::hal::pac::interrupt;

use fugit::RateExtU32;

use ina226;

use heapless::String;

// GPIO traits
use embedded_hal::digital::OutputPin;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// Pull in any important traits
use rp_pico::hal::prelude::*;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use rp_pico::hal::pac;

// A shorter alias for the Hardware Abstraction Layer, which provides
// higher-level drivers.
use rp_pico::hal;

// USB Device support
use usb_device::{class_prelude::*, prelude::*};

// USB Communications Class Device support
use usbd_serial::SerialPort;

/// The USB Device Driver (shared with the interrupt).
static mut USB_DEVICE: Option<UsbDevice<hal::usb::UsbBus>> = None;

/// The USB Bus Driver (shared with the interrupt).
static mut USB_BUS: Option<UsbBusAllocator<hal::usb::UsbBus>> = None;

/// The USB Serial Device Driver (shared with the interrupt).
static mut USB_SERIAL: Option<SerialPort<hal::usb::UsbBus>> = None;

/// This function is called whenever the USB Hardware generates an Interrupt
/// Request.
///
/// We do all our USB work under interrupt, so the main thread can continue on
/// knowing nothing about USB.
#[allow(non_snake_case)]
#[interrupt]
unsafe fn USBCTRL_IRQ() {

    // Grab the global objects. This is OK as we only access them under interrupt.
    let usb_dev = USB_DEVICE.as_mut().unwrap();
    let serial = USB_SERIAL.as_mut().unwrap();

    // Poll the USB driver with all of our supported USB Classes
    if usb_dev.poll(&mut [serial]) {
        let mut buf = [0u8; 64];
        match serial.read(&mut buf) {
            Err(_e) => {
                // Do nothing
            }
            Ok(0) => {
                // Do nothing
            }
            Ok(count) => {
                // Convert to upper case
                buf.iter_mut().take(count).for_each(|b| {
                    b.make_ascii_uppercase();
                });

                // Send back to the host
                let mut wr_ptr = &buf[..count];
                while !wr_ptr.is_empty() {
                    let _ = serial.write(wr_ptr).map(|len| {
                        wr_ptr = &wr_ptr[len..];
                    });
                }
            }
        }
    }
}


/// Entry point to our bare-metal application.
///
/// The `#[entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables are initialised.
///
/// The function configures the RP2040 peripherals, then blinks the LED in an
/// infinite loop.
#[entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    //
    // The default is to generate a 125 MHz system clock
    let clocks = hal::clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    // Set up the USB driver
    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));
    unsafe {
        // Note (safety): This is safe as interrupts haven't been started yet
        USB_BUS = Some(usb_bus);
    }

    // Grab a reference to the USB Bus allocator. We are promising to the
    // compiler not to take mutable access to this global variable whilst this
    // reference exists!
    let bus_ref = unsafe { USB_BUS.as_ref().unwrap() };

    // Set up the USB Communications Class Device driver
    let serial = SerialPort::new(bus_ref);
    unsafe {
        USB_SERIAL = Some(serial);
    }

    // Create a USB device with a fake VID and PID
    let usb_dev = UsbDeviceBuilder::new(bus_ref, UsbVidPid(0x16c0, 0x27dd))
        .strings(&[StringDescriptors::default()
            .manufacturer("Fake company")
            .product("Serial port")
            .serial_number("TEST")])
        .unwrap()
        .device_class(2) // from: https://www.usb.org/defined-class-codes
        .build();
    unsafe {
        // Note (safety): This is safe as interrupts haven't been started yet
        USB_DEVICE = Some(usb_dev);
    }

    // Enable the USB interrupt
    unsafe {
        pac::NVIC::unmask(hal::pac::Interrupt::USBCTRL_IRQ);
    };

    // No more USB code after this point in main! We can do anything we want in
    // here since USB is handled in the interrupt - let's blink an LED!

    // The delay object lets us wait for specified amounts of time (in
    // milliseconds)
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins up according to their function on this particular board
    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Configure two pins as being I2C, not GPIO
    let sda_pin: hal::gpio::Pin<_, hal::gpio::FunctionI2C, _> = pins.gpio0.reconfigure();
    let scl_pin: hal::gpio::Pin<_, hal::gpio::FunctionI2C, _> = pins.gpio1.reconfigure();

    // Create the I2C driver, using the two pre-configured pins. This will fail
    // at compile time if the pins are in the wrong mode, or if this I2C
    // peripheral isn't available on these pins!
    let i2c = hal::I2C::i2c0(
        pac.I2C0,
        sda_pin,
        scl_pin,
        400.kHz(),
        &mut pac.RESETS,
        &clocks.peripheral_clock,
    );

    let mut led_pin1 = pins.gpio10.into_push_pull_output();
    let mut led_pin2 = pins.gpio11.into_push_pull_output();

    let mut vbus_out = pins.gpio2.into_push_pull_output();
    let _ = vbus_out.set_low(); // enable VBUS for now.

    let mut ina226 = ina226::INA226::new(i2c, ina226::DEFAULT_ADDRESS);

    // 50mOhm shunt, 1A expected max current.
    let _ = ina226.callibrate(0.05f64, 1.0f64);

    let id = ina226.manufacturer_id().unwrap_or(0);

    let mut n_ilow = 0u32;

    // Blink the LED at 1 Hz
    loop {

        // led pattern TODO remove

        led_pin1.set_high().unwrap();
        led_pin2.set_low().unwrap();
        delay.delay_ms(10);
        led_pin1.set_high().unwrap();
        led_pin2.set_high().unwrap();
        delay.delay_ms(200);
        led_pin1.set_low().unwrap();
        led_pin2.set_high().unwrap();
        delay.delay_ms(10);
        led_pin1.set_high().unwrap();
        led_pin2.set_high().unwrap();
        delay.delay_ms(200);

        // read from current shunt

        let shunt_voltage_uv = ina226.shunt_voltage_microvolts().unwrap_or(0f64);
        let bus_voltage_mv   = ina226.bus_voltage_millivolts().unwrap_or(0f64);
        let current_a        = ina226.current_amps().unwrap_or(None).unwrap_or(0f64);

        // auto-reset logic

        if current_a < 0.1f64 {
            n_ilow += 1;
        }

        if n_ilow > 3 {
            let _ = vbus_out.set_high();
            for _ in 0..10 {
                let _ = led_pin1.set_high();
                let _ = led_pin2.set_low();
                delay.delay_ms(25);
                let _ = led_pin1.set_low();
                let _ = led_pin2.set_high();
                delay.delay_ms(25);
            }
            let _ = vbus_out.set_low();
            n_ilow = 0;
        }

        unsafe {
            use core::fmt::Write;
            let serial = USB_SERIAL.as_mut().unwrap();
            let usb_dev = USB_DEVICE.as_mut().unwrap();
            let mut text: String<128> = String::new();
            writeln!(&mut text, "INA226  ID:   {:x}", id).unwrap();
            writeln!(&mut text, "SHUNT   uV: {:.6}", shunt_voltage_uv).unwrap();
            writeln!(&mut text, "BUS     mV: {:.6}", bus_voltage_mv).unwrap();
            writeln!(&mut text, "CURRENT  A: {:.6}", current_a).unwrap();
            let _ = serial.write(text.as_bytes());
            usb_dev.poll(&mut [serial]);
        }
    }
}

// End of file
