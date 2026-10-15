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

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

/*                  Xiao Esp32-c3 Complete Circuit (V0 -> 2 Potentioneter)
*
*                                 Front
*
*                                |     |
*                      +---------+USB C+---------+
*     O                |                 Char LED|
*     "               O| D0|A0|G2        VBUS(5V)|O++            *
*     "               O| D1|A1|G3          GND   |O++          -[ Servo0 ]-
*  [Pote0]     Pote0--O| D2|A2|G4        3.3V Out|O             [       ]
*  [V R G]     Pote1--O| D3|A3|G5         D10|G10|O             [G D V]
*   V D2              O| D4|I2_SDA|G6      D9|G9 |O--LED          D7 V
*                     O| D5|I2_SDL|G7      D8|G8 |O--Servo1_D
*     O               O| D6|G21            D7|G7 |O--Servo0_D
*     "                |                         |
*     "                | B:[] G9                 |               *
*  [Pote1]             +--------------------------+            -[ Servo1 ]-
*  [V R G]                                                      [       ]
*   V D1                           100ohm       100ohm          [G D V]
*                              D9--~~~---+   D9-~~~+              D8 V
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

extern crate alloc;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    // generator version: 1.3.0
    // generator parameters: --chip esp32c3 -o  -o esp32c3-mini-1 -o unstable-hal -o alloc -o embassy

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    // The following pins are used to bootstrap the chip. They are available
    // for use, but check the datasheet of the module for more information on them.
    // - GPIO2
    // - GPIO8
    // - GPIO9
    // These GPIO pins are in use by some feature of the module and should not be used.
    let _ = peripherals.GPIO11;
    let _ = peripherals.GPIO12;
    let _ = peripherals.GPIO13;
    let _ = peripherals.GPIO14;
    let _ = peripherals.GPIO15;
    let _ = peripherals.GPIO16;
    let _ = peripherals.GPIO17;

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 66320);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let sw_interrupt =
        esp_hal::interrupt::software::SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_interrupt.software_interrupt0);

    // Set GPIO0 as an output, and set its state high initially.
    let mut led = esp_hal::gpio::Output::new(
        peripherals.GPIO10,
        esp_hal::gpio::Level::High,
        esp_hal::gpio::OutputConfig::default(),
    );

    // TODO: Spawn some tasks
    let _ = spawner;

    loop {
        Timer::after(Duration::from_secs(1)).await;
        led.toggle();
    }

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v1.1.0/examples
}
