#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_hal::clock::CpuClock;
use esp_hal::timer::timg::TimerGroup;
use log::{error, info};

use esp_hal::delay::Delay;
use esp_hal::main;

// LEDC
use esp_hal::gpio::DriveMode;
use esp_hal::ledc::channel::ChannelIFace;
use esp_hal::ledc::timer::TimerIFace;
use esp_hal::ledc::{Ledc, LowSpeed, channel, timer};
use esp_hal::time::Rate;

use embedded_hal::pwm::SetDutyCycle;

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
*     O               O| D6|G21            D7|G7 |O--Servo0_D
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
*                GyR--O| D6|G21            D7|G7 |O--Sservo1_D
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

struct Servo {
    pub id: u8,
    pin: esp_hal::peripherals::GPIO<'static>,
    channel: esp_hal::ledc::channel::Channel<'static, esp_hal::ledc::LowSpeed>,
}

impl Servo {
    pub fn new(
        id: u8,
        peripherals: &mut esp_hal::peripherals::Peripherals,
        servo_pin_numb: u8,
        ledc: &esp_hal::ledc::Ledc,
    ) -> Self {
        let mut channel0 =
            ledc.channel(esp_hal::ledc::channel::Number::Channel0, peripherals.GPIO8);
        channel0
            .configure(esp_hal::ledc::channel::config::Config {
                timer: &ledc,
                duty_pct: 10,
                drive_mode: esp_hal::gpio::DriveMode::PushPull,
            })
            .unwrap();

        let delay = esp_hal::delay::Delay::new();

        let max_duty_cycle = channel0.max_duty_cycle() as u32;

        // Minimum duty (2.5%)
        // For 12bit -> 25 * 4096 /1000 => ~ 102
        let min_duty = (25 * max_duty_cycle) / 1000;
        // Maximum duty (12.5%)
        // For 12bit -> 125 * 4096 /1000 => 512
        let max_duty = (125 * max_duty_cycle) / 1000;
        // 512 - 102 => 410
        let duty_gap = max_duty - min_duty;
        todo!()
    }

    fn set_angle(angle: u8) {
        todo!()
    }
}

struct Arms {
    left: Servo,
    right: Servo,
}

// TODO: change to radian if needed
fn duty_from_angle(deg: u32, min_duty: u32, duty_gap: u32) -> u16 {
    let duty = min_duty + ((deg * duty_gap) / 180);
    duty as u16
}

#[embassy_executor::task]
async fn potent_task(
    poten0_pin: esp_hal::peripherals::GPIO4<'static>,
    poten1_pin: esp_hal::peripherals::GPIO5<'static>,
    adc2: esp_hal::peripherals::ADC2<'static>,
    arms: Arms,
) {
    // From https://esp32.implrust.com/core-concepts/adc/adc-in-esp32.html
    // Attenuation	    Measurable input voltage range
    // 0   dB	          00  mV ~ 950 mV
    // 2.5 dB	          100 mV ~ 1250 mV
    // 6   dB	          150 mV ~ 1750 mV
    // 11  dB	          150 mV ~ 2450 mV

    let mut adc2_config = esp_hal::analog::adc::AdcConfig::new();
    let mut poten0 = adc2_config.enable_pin(poten0_pin, esp_hal::analog::adc::Attenuation::_11dB);
    let mut poten1 = adc2_config.enable_pin(poten1_pin, esp_hal::analog::adc::Attenuation::_6dB);
    let mut adc2 = esp_hal::analog::adc::Adc::new(adc2, adc2_config);

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

        Timer::after(Duration::from_millis(75)).await;
    }
}

async fn circuitv0(spawner: Spawner, peripherals: esp_hal::peripherals::Peripherals) -> ! {
    // 1 - Configure read button on D9/G9 and turn on LED
    // 2 - ADC on each Potentiometer and turn servo according to values

    let led_button_gp = peripherals.GPIO9;
    let led_out_gp = peripherals.GPIO10;

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0, peripherals.FROM_CPU_INTR0);

    // Set GPIO9 as an output, and set its state high initially.
    let mut led_out = esp_hal::gpio::Output::new(
        led_out_gp,
        esp_hal::gpio::Level::High,
        esp_hal::gpio::OutputConfig::default(),
    );

    let poten0 = esp_hal::peripherals::GPIO4;
    let poten1 = esp_hal::peripherals::GPIO5;
    let adc2_peripheral = peripherals.ADC2;
    // spawner.spawn(check_joypad(urx, ury, adc2_peripheral)).ok();

    todo!()
    // spawner.spawn();

    // spawner.spawn(
    //     async move {
    //         Timer::after_millis(250).await;
    //     }
    //     .await,
    // );

    // TODO: Spawn some tasks
    // let _ = spawner;
    //
    // loop {
    //
    //     spawner.spawn();
    //
    //     Timer::after(Duration::from_secs(1)).await;
    //     led.toggle();
    // }
}

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
    //
    // // The following pins are used to bootstrap the chip. They are available
    // // for use, but check the datasheet of the module for more information on them.
    // // - GPIO2
    // // - GPIO8
    // // - GPIO9
    // // These GPIO pins are in use by some feature of the module and should not be used.
    // let _gpio11 = peripherals.GPIO11;
    // let _gpio12 = peripherals.GPIO12;
    // let _gpio13 = peripherals.GPIO13;
    // let _gpio14 = peripherals.GPIO14;
    // let _gpio15 = peripherals.GPIO15;
    // let _gpio16 = peripherals.GPIO16;
    // let _gpio17 = peripherals.GPIO17;
    //
    // esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 66320);
    //
    // let timg0 = TimerGroup::new(peripherals.TIMG0);
    // esp_rtos::start(timg0.timer0, peripherals.FROM_CPU_INTR0);
    //
    // info!("Embassy initialized!");
    //
    // // TODO: Spawn some tasks
    // let _ = spawner;
    //
    // loop {
    //     info!("Hello world!");
    //     Timer::after(Duration::from_secs(1)).await;
    // }

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    let mut servo = peripherals.GPIO8;
    let ledc = Ledc::new(peripherals.LEDC);

    let mut hstimer0 = ledc.timer::<LowSpeed>(timer::Number::Timer0);
    hstimer0
        .configure(timer::config::Config {
            duty: timer::config::Duty::Duty12Bit,
            clock_source: timer::LSClockSource::APBClk,
            frequency: Rate::from_hz(50),
        })
        .unwrap();

    let mut channel0 = ledc.channel(channel::Number::Channel0, servo.reborrow());
    channel0
        .configure(channel::config::Config {
            timer: &hstimer0,
            duty_pct: 10,
            drive_mode: DriveMode::PushPull,
        })
        .unwrap();

    let delay = Delay::new();

    let max_duty_cycle = channel0.max_duty_cycle() as u32;

    // Minimum duty (2.5%)
    // For 12bit -> 25 * 4096 /1000 => ~ 102
    let min_duty = (25 * max_duty_cycle) / 1000;
    // Maximum duty (12.5%)
    // For 12bit -> 125 * 4096 /1000 => 512
    let max_duty = (125 * max_duty_cycle) / 1000;
    // 512 - 102 => 410
    let duty_gap = max_duty - min_duty;

    loop {
        for deg in 0..=180 {
            let duty = duty_from_angle(deg, min_duty, duty_gap);
            channel0.set_duty_cycle(duty).unwrap();
            delay.delay_millis(10);
        }
        delay.delay_millis(500);

        for deg in (0..=180).rev() {
            let duty = duty_from_angle(deg, min_duty, duty_gap);
            channel0.set_duty_cycle(duty).unwrap();
            delay.delay_millis(10);
        }
        delay.delay_millis(500);
    }

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v1.2.2/examples
}
