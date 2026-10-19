#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use core::cell::RefCell;

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_hal::clock::CpuClock;
use esp_hal::timer::timg::TimerGroup;
use log::{error, info};

use esp_hal::delay::Delay;
use esp_hal::main;

// LEDC
use esp_hal::gpio::{DriveMode, Input, Output};
use esp_hal::ledc::channel::ChannelIFace;
use esp_hal::ledc::timer::TimerIFace;
use esp_hal::ledc::{Ledc, LowSpeed, channel, timer};
use esp_hal::time::Rate;

use embedded_hal::pwm::SetDutyCycle;
use mpu6050_dmp::sensor::Mpu6050;

#[panic_handler]
fn panic(panic_info: &core::panic::PanicInfo) -> ! {
    error!("{}", panic_info);
    loop {}
}

extern crate alloc;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

/*                  Xiao Esp32-c3 Complete Circuit (V0 -> 2 Potentiometer)
*
*                                 Front
*
*                                |     |
*                      +---------+USB C+---------+
*     O                |                 Char LED|
*     "               O| D0|A0|G2        VBUS(5V)|O++            *
*     "               O| D1|A1|G3          GND   |O++          -[ Servo0 ]-
*  [Pote0]     Pote0--O| D2|A2|G4        3.3V Out|O             [       ]
*  [V R G]     Pote1--O| D3|A3|G5         D10|G10|O--D10        [G D V]
*   V D2              O| D4|I2_SDA|G6      D9|G9 |O               D7 V
*                     O| D5|I2_SDL|G7      D8|G8 |O--Servo1_D
*     O               O| D6|G21            D7|G20|O--Servo0_D
*     "                |                         |
*     "                | B:[] G9                 |               *
*  [Pote1]             +---|----------------------+            -[ Servo1 ]-
*  [V R G]    [LED_BUTT]O--+                                    [       ]
*   V D1                           100ohm       100ohm          [G D V]
*                              D10-~~~---+   D10~~~+              D8 V
* - Pote0 moves Servo0                  [V G]     [V G]
* - Pote1 moves Servo1                  [LED0]    [LED1]
* - D9/Gpio9 turns Leds
*
* =======================================================================
*  Xiao Esp32-c3 Complete Circuit (V1 -> 6 Servos Final)
*
*    {Batt}                       Front                -[Bservo0]-  -[Bservo1]-
*    { Li }                                             [G D  V]    [G D    V]
*    {3.5V}                      |     |                  D10 B5V   [  D1 B5V]
*   {B3V BO}           +---------+USB C+---------+
*     |                |                 Char LED|
*     ^       BatRead-O| D0|A0|G2        VBUS(5V)|O++
*     ^ Buck  Bservo1-O| D1|A1|G3          GND   |O++          -[Sservo0]-  Sservo2
*     ^ Conv  Sservo2-O| D2|A2|G4        3.3V Out|O++           [      ]      D2
*     ^ 3-5V  Sservo3-O| D3|A3|G5         D10|G10|O--Bservo1_D  [G D V]
*     |          I2C--O| D4|I2_SDA|G6      D9|G9 |O--Bat_Val      D7 B5V
*     B5V        I2C--O| D5|I2_SDL|G7      D8|G8 |O--Sservo0_D
*                GyR--O| D6|G21            D7|G20|O--Sservo1_D
*                      |            Bin Bout     |
*                      | B:[] G9   [B3V] [BO]+   |
*  [MPU0]      [MPU1]  +-----------|(cera 0.1uf)-|             -[Sservo1]-  Sservo3
*  [VCC]O-3v--O[VCC]               |(eletro 1mF)-|              [      ]     D3
*  [GND]O--G--O[GND]                                            [G D V]
*  [SDL]O-D5--O[SDL]                                              D8 B5V
*  [SDA]O-D4--O[SDA]
*  [ADD]O      [ADD]O-3.3V            HAND Button Controls
*                                     [Gyro Reset]O-D6
*      100ohm                         [LED Toggle]O-LED_G_Switch
*   3V--~~~---+ +--LED_Switch-+       [Read Batt ]O-A0   [VAL DISPLAY]O-D10
*          | [V G]      +=[V G]       [Shutdown]---Disconnects Ground----
*          | [LED0]     | [LED1]
*          +------------+
*
* - Readings from MPU0 controls Arm0 -> Bs0, ss0, ss1 with inverse kinematics
* - Readings from MPU1 controls Arm1 -> Bs1, ss2, ss3 with inverse kinematics
* - Gyro reset: moves arms (servos) to start pos + makes hand pos start pos
* - LED toggle is a toggle button that grounds the LEDs
* - Read Batt will show the battery percentage on a simple display
*/

// TODO: change to radian if needed
fn duty_from_angle(deg: u32, min_duty: u32, duty_gap: u32) -> u16 {
    let duty = min_duty + ((deg * duty_gap) / 180);
    duty as u16
}

static BUTTON: critical_section::Mutex<RefCell<Option<Input>>> =
    critical_section::Mutex::new(RefCell::new(None));

static BUTTON_OUT: critical_section::Mutex<RefCell<Option<Output>>> =
    critical_section::Mutex::new(RefCell::new(None));

// Version 0 --> Potentiometer
#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    // generator version: 1.4.0
    // generator parameters: -o esp32c3 -o esp32c3-mini-1 -o unstable-hal -o alloc -o embassy -o log -o neovim-classic
    //
    // esp_println::logger::init_logger_from_env();
    //
    // let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    // let peripherals = esp_hal::init(config);

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 66320);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0, peripherals.FROM_CPU_INTR0);
    info!("Embassy initialized!");

    // I2C handling
    let i2c_bus = esp_hal::i2c::master::I2c::new(
        peripherals.I2C0,
        esp_hal::i2c::master::Config::default().with_frequency(Rate::from_khz(400)),
    )
    .unwrap()
    .with_scl(peripherals.GPIO7)
    .with_sda(peripherals.GPIO6);

    let mut mpu = Mpu6050::new(i2c_bus, mpu6050_dmp::address::Address::default()).unwrap();
    let mut delay = embassy_time::Delay;
    mpu.initialize_dmp(&mut delay).unwrap();

    let i2c_bus = info!("MPU6050-DMP Sensor Initialized");

    // Initialize the Digital Motion Processor (DMP) firmware
    // The DMP provides enhanced motion processing capabilities and sensor fusion
    // This step resets the DMP to its default state and loads the firmware
    // info!("Initializing DMP Firmware");
    // mpu.initialize_dmp(&mut delay).await.unwrap();
    // info!("DMP Firmware Initialized");

    // Read raw accelerometer data (uncalibrated)
    // The accelerometer measures linear acceleration in three axes (X, Y, Z)
    // Values will be imprecise until calibration is performed
    // let accel_data = sensor.accel().await.unwrap();
    // info!(
    //     "Accelerometer [mg]: x={}, y={}, z={}",
    //     accel_data.x() as i32,
    //     accel_data.y() as i32,
    //     accel_data.z() as i32
    // );

    // LED interrupt handling -> in GPIO9, out GPIO10
    let mut io = esp_hal::gpio::Io::new(peripherals.IO_MUX);
    io.set_interrupt_handler(handler);

    let led = esp_hal::gpio::Output::new(
        peripherals.GPIO10,
        esp_hal::gpio::Level::Low,
        esp_hal::gpio::OutputConfig::default(),
    );

    let config = esp_hal::gpio::InputConfig::default().with_pull(esp_hal::gpio::Pull::Up);
    let mut button = esp_hal::gpio::Input::new(peripherals.GPIO9, config);

    critical_section::with(|cs| {
        button.listen(esp_hal::gpio::Event::FallingEdge);
        BUTTON_OUT.borrow_ref_mut(cs).replace(led);
        BUTTON.borrow_ref_mut(cs).replace(button)
    });

    let mut servo0 = peripherals.GPIO20;
    let mut servo1 = peripherals.GPIO8;
    let ledc = Ledc::new(peripherals.LEDC);

    let mut hstimer0 = ledc.timer::<LowSpeed>(timer::Number::Timer0);
    hstimer0
        .configure(timer::config::Config {
            duty: timer::config::Duty::Duty12Bit,
            clock_source: timer::LSClockSource::APBClk,
            frequency: Rate::from_hz(50),
        })
        .unwrap();

    let mut channel0 = ledc.channel(channel::Number::Channel0, servo0.reborrow());
    channel0
        .configure(channel::config::Config {
            timer: &hstimer0,
            duty_pct: 10,
            drive_mode: DriveMode::PushPull,
        })
        .unwrap();

    let mut hstimer1 = ledc.timer::<LowSpeed>(timer::Number::Timer1);
    hstimer1
        .configure(timer::config::Config {
            duty: timer::config::Duty::Duty12Bit,
            clock_source: timer::LSClockSource::APBClk,
            frequency: Rate::from_hz(50),
        })
        .unwrap();

    let mut channel1 = ledc.channel(channel::Number::Channel1, servo1.reborrow());
    channel1
        .configure(channel::config::Config {
            timer: &hstimer1,
            duty_pct: 10,
            drive_mode: DriveMode::PushPull,
        })
        .unwrap();

    let max_duty_cycle = channel0.max_duty_cycle() as u32;

    // Minimum duty (2.5%)
    // For 12bit -> 25 * 4096 /1000 => ~ 102
    let min_duty = (25 * max_duty_cycle) / 1000;
    // Maximum duty (12.5%)
    // For 12bit -> 125 * 4096 /1000 => 512
    let max_duty = (125 * max_duty_cycle) / 1000;
    // 512 - 102 => 410
    let duty_gap = max_duty - min_duty;

    // From https://esp32.implrust.com/core-concepts/adc/adc-in-esp32.html
    // Attenuation	    Measurable input voltage range
    // 0   dB	          00  mV ~ 950 mV
    // 2.5 dB	          100 mV ~ 1250 mV
    // 6   dB	          150 mV ~ 1750 mV
    // 11  dB	          150 mV ~ 2450 mV

    let mut adc2_config = esp_hal::analog::adc::AdcConfig::new();
    let mut poten0 =
        adc2_config.enable_pin(peripherals.GPIO4, esp_hal::analog::adc::Attenuation::_11dB);
    let mut poten1 =
        adc2_config.enable_pin(peripherals.GPIO5, esp_hal::analog::adc::Attenuation::_11dB);
    let mut adc2 = esp_hal::analog::adc::Adc::new(peripherals.ADC2, adc2_config);

    esp_println::println!("Starting Potentiometer task, values sent to servos every 75ms");

    loop {
        let poten0_val: u16 = nb::block!(adc2.read_oneshot(&mut poten0)).unwrap();
        let poten1_val: u16 = nb::block!(adc2.read_oneshot(&mut poten1)).unwrap();

        let min_angle = 0; // Change to radians if you want
        let max_angle = 180; // Change to radians if you want

        let servo0_pos = (((poten0_val - min_angle) * (max_angle - min_angle))
            / (u16::MAX - min_angle))
            + min_angle;
        let servo1_pos = (((poten1_val - min_angle) * (max_angle - min_angle))
            / (u16::MAX - min_angle))
            + min_angle;

        esp_println::println!(
            "Poten0:{poten0_val} -> Servo0: {servo0_pos} || Poten1:{poten0_val} -> Servo1: {servo1_pos}",
        );

        // Send it to servos to do their thing
        channel0
            .set_duty_cycle(duty_from_angle(servo0_pos as u32, min_duty, duty_gap))
            .unwrap();
        channel1
            .set_duty_cycle(duty_from_angle(servo1_pos as u32, min_duty, duty_gap))
            .unwrap();
        Timer::after(Duration::from_millis(50)).await;

        Timer::after(Duration::from_millis(250)).await;
    }

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v1.2.2/examples
}

#[esp_hal::handler]
// #[esp_hal::ram]
fn handler() {
    esp_println::println!("GPIO Interrupt");
    if critical_section::with(|cs| {
        BUTTON
            .borrow_ref_mut(cs)
            .as_mut()
            .unwrap()
            .is_interrupt_set()
    }) {
        esp_println::println!("Button was the source of the interrupt");
        critical_section::with(|cs| {
            if let Some(out) = BUTTON_OUT.borrow_ref_mut(cs).as_mut() {
                esp_println::println!("LED was low setting it to high");
                if out.is_set_low() {
                    out.set_high();
                } else {
                    esp_println::println!("LED was high setting it to low");
                    out.set_low();
                }
            };
        });
    } else {
        esp_println::println!("Button was not the source of the interrupt");
    }

    critical_section::with(|cs| {
        BUTTON
            .borrow_ref_mut(cs)
            .as_mut()
            .unwrap()
            .clear_interrupt()
    });
}
