#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use core::cell::RefCell;
use core::mem::MaybeUninit;

use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_time::{Duration, Instant, Timer};
use esp_hal::clock::CpuClock;
use esp_hal::timer::timg::TimerGroup;
use log::{error, info};

// LEDC
use esp_hal::gpio::{DriveMode, Input, Output};
use esp_hal::ledc::channel::{ChannelIFace, Number};
use esp_hal::ledc::timer::TimerIFace;
use esp_hal::ledc::{self, Ledc, LowSpeed, channel, timer};
use esp_hal::time::Rate;

use embedded_hal::pwm::SetDutyCycle;
use mpu6050_async::Mpu6050;
use static_cell::StaticCell;

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
*     ^ 3-5V  Sservo3-O| D3|A3|G5         D10|G10|O--Bservo0_D  [G D V]
*     |          I2C--O| D4|I2_SDA|G6      D9|G9 |O--Bat_Val      D7 B5V
*     B5V        I2C--O| D5|I2_SDL|G7      D8|G8 |O--Sservo1_D
*                GyR--O| D6|G21            D7|G20|O--Sservo0_D
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

static BUTTON_BATT: critical_section::Mutex<RefCell<Option<Input>>> =
    critical_section::Mutex::new(RefCell::new(None));

static BUTTON_BATT_OUT: critical_section::Mutex<RefCell<Option<Output>>> =
    critical_section::Mutex::new(RefCell::new(None));

static BUTTON_GYRO_RESET: critical_section::Mutex<RefCell<Option<Input>>> =
    critical_section::Mutex::new(RefCell::new(None));

static TO_GYRO_RESET: critical_section::Mutex<RefCell<bool>> =
    critical_section::Mutex::new(RefCell::new(false));

type I2cBus =
    embassy_sync::mutex::Mutex<NoopRawMutex, esp_hal::i2c::master::I2c<'static, esp_hal::Async>>;
type SharedMpu = Mpu6050<
    embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice<
        'static,
        NoopRawMutex,
        esp_hal::i2c::master::I2c<'static, esp_hal::Async>,
    >,
>;
static I2C_BUS: StaticCell<I2cBus> = StaticCell::new();

fn set_interrupts<'a>(
    gyro_gpio: impl esp_hal::gpio::InputPin + 'static,
    batt_gpio: impl esp_hal::gpio::InputPin + 'static,
) {
    let mut gyro_reset_button = esp_hal::gpio::Input::new(
        gyro_gpio,
        esp_hal::gpio::InputConfig::default().with_pull(esp_hal::gpio::Pull::Up),
    );

    let mut batt_button = esp_hal::gpio::Input::new(
        batt_gpio,
        esp_hal::gpio::InputConfig::default().with_pull(esp_hal::gpio::Pull::Up),
    );

    critical_section::with(|cs| {
        batt_button.listen(esp_hal::gpio::Event::FallingEdge);
        BUTTON_BATT.borrow_ref_mut(cs).replace(batt_button);

        gyro_reset_button.listen(esp_hal::gpio::Event::FallingEdge);
        BUTTON_GYRO_RESET
            .borrow_ref_mut(cs)
            .replace(gyro_reset_button)
    });
}

/// Hold the Accelerometer flat with its z pointing up for about 2s
async fn calibrate_accel(mpu: &mut SharedMpu) -> [f32; 3] {
    const SAMPLES: usize = 200;
    let mut sum = [0.0f32; 3];

    for _ in 0..SAMPLES {
        let a = mpu.get_acc().await.unwrap();
        sum[0] += a.x;
        sum[1] += a.y;
        sum[2] += a.z;
        Timer::after(Duration::from_millis(10)).await;
    }

    let n = SAMPLES as f32;
    [sum[0] / n, sum[1] / n, sum[2] / n - 1.0] // Z should read 1 g, not 0
}

fn cosf_angle(x: f32) -> f32 {
    libm::cosf(x * 3.141592 / 180.0)
}

fn sinf_angle(x: f32) -> f32 {
    libm::sinf(x * 3.141592 / 180.0)
}

/// Brute forces all possible servo angles to find the closest to the desired positions
/// the ranges are [min, max, step] all in degrees (max is 256)
fn angles_from_pos(
    curr_angles: [u8; 3],
    target_pos: [f32; 3],
    servo0_range: [u8; 3],
    servo1_range: [u8; 3],
    servo2_range: [u8; 3],
) -> [u8; 3] {
    // let mut best_pos = None;
    let mut best_angles = None;
    let mut best_pos_diff = None;
    let mut best_work_diff = None;

    // size in m
    let l0x = 0.01; // 1cm
    let l0y = 0.01; // 1cm
    let l1z = 0.05; // 5cm
    let l2x = 0.07; // 5cm

    // Changing angles of servo 0 costs a lot power
    // also servo 1 costs more thant servo 2
    let servo0_work_mul = 5;
    let servo1_work_mul = 2;

    // let curr_pos_x =
    //     cosf_angle(curr_angles[2] as f32) * l2x + l0x * cosf_angle(curr_angles[0] as f32);
    // let curr_pos_y = l0y + sinf_angle(curr_angles[1] as f32) * l1z;
    // let curr_pos_z = sinf_angle(curr_angles[2] as f32) * l2x
    //     + l1z * cosf_angle(curr_angles[1] as f32)
    //     + l0x * sinf_angle(curr_angles[0] as f32);

    let diff_pos_threshold = 10.0; // If difference is below this amount then they are basically the
    // same. Basically a deadzone threshold

    for theta0 in (servo0_range[0]..servo0_range[1]).step_by(servo0_range[2] as usize) {
        for theta1 in (servo1_range[0]..servo1_range[1]).step_by(servo1_range[2] as usize) {
            for theta2 in (servo2_range[0]..servo2_range[1]).step_by(servo2_range[2] as usize) {
                let combo_x = cosf_angle(theta2 as f32) * l2x + l0x * cosf_angle(theta0 as f32);
                let combo_y = l0y + sinf_angle(theta1 as f32) * l1z;
                let combo_z = sinf_angle(theta2 as f32) * l2x
                    + l1z * cosf_angle(theta1 as f32)
                    + l0x * sinf_angle(theta0 as f32);

                let curr_work = (theta0 as i16 - curr_angles[0] as i16).abs() * servo0_work_mul
                    + (theta1 as i16 - curr_angles[1] as i16).abs() * servo1_work_mul
                    + (theta2 as i16 - curr_angles[2] as i16).abs();

                let combo_to_target_pos_diff = libm::powf(
                    libm::powf(combo_x - target_pos[0], 2.0)
                        + libm::powf(combo_y - target_pos[1], 2.0)
                        + libm::powf(combo_z - target_pos[2], 2.0),
                    0.5,
                );

                let (Some(ref mut best_work_diff), Some(ref mut best_pos_diff)) =
                    (best_work_diff, best_pos_diff)
                else {
                    best_angles = Some([theta0, theta1, theta2]);
                    best_pos_diff = Some(combo_to_target_pos_diff);
                    best_work_diff = Some(curr_work);
                    continue;
                };

                // TODO: Pick a point that has less work if within threshold
                // something like that

                // This new point beats the old one and is cheaper
                if combo_to_target_pos_diff <= *best_pos_diff {
                    best_angles = Some([theta0, theta1, theta2]);
                    *best_pos_diff = combo_to_target_pos_diff;
                    *best_work_diff = curr_work;
                }
            }
        }
    }

    // TODO: fix unwrap
    best_angles.unwrap()
}

fn make_servo<'a>(
    gpio: impl esp_hal::gpio::OutputPin + 'a,
    timer: &'a ledc::timer::Timer<'a, LowSpeed>,
    channel_numb: channel::Number,
    ledc: &mut Ledc<'a>,
) -> ledc::channel::Channel<'a, LowSpeed> {
    let mut channel = ledc.channel(channel_numb, gpio);
    channel
        .configure(channel::config::Config {
            timer: timer,
            duty_pct: 10,
            drive_mode: DriveMode::PushPull,
        })
        .unwrap();
    channel
}

fn angle_on_servo<'a>(servo: &mut esp_hal::ledc::channel::Channel<'a, LowSpeed>, angle: u8) {
    let max_duty_cycle = servo.max_duty_cycle() as u32;
    // Minimum duty (2.5%)
    // For 12bit -> 25 * 4096 /1000 => ~ 102
    let min_duty = (25 * max_duty_cycle) / 1000;
    // Maximum duty (12.5%)
    // For 12bit -> 125 * 4096 /1000 => 512
    let max_duty = (125 * max_duty_cycle) / 1000;
    // 512 - 102 => 410
    let duty_gap = max_duty - min_duty;

    servo
        .set_duty_cycle(duty_from_angle(angle as u32, min_duty, duty_gap))
        .unwrap();
}

// Version 1 --> Accs and All servos
#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    // generator version: 1.4.0
    // generator parameters: -o esp32c3 -o esp32c3-mini-1 -o unstable-hal -o alloc -o embassy -o log -o neovim-classic
    esp_println::logger::init_logger_from_env();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 66320);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0, peripherals.FROM_CPU_INTR0);

    let mut io = esp_hal::gpio::Io::new(peripherals.IO_MUX);
    io.set_interrupt_handler(handler);

    info!("Basic timing and IO init done!");

    let i2c_bus = esp_hal::i2c::master::I2c::new(
        peripherals.I2C0,
        esp_hal::i2c::master::Config::default().with_frequency(Rate::from_khz(400)),
    )
    .unwrap()
    .with_scl(peripherals.GPIO7)
    .with_sda(peripherals.GPIO6)
    .into_async();

    let i2c_bus: &'static I2cBus = I2C_BUS.init(embassy_sync::mutex::Mutex::new(i2c_bus));

    let mut acc_mpu_0 = mpu6050_async::Mpu6050::new_with_addr(
        embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice::new(i2c_bus),
        mpu6050_async::device::DEFAULT_SLAVE_ADDR,
    );

    let mut acc_mpu_1 = mpu6050_async::Mpu6050::new_with_addr(
        embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice::new(i2c_bus),
        mpu6050_async::device::DEFAULT_SLAVE_ADDR + 1,
    );

    let mut delay = embassy_time::Delay;
    acc_mpu_0.init(&mut delay).await.unwrap();
    acc_mpu_1.init(&mut delay).await.unwrap();

    let acc_0 = acc_mpu_0.get_acc().await.unwrap();
    info!(
        "Accelerometer 0 init done! Current x:{:.2}mg, y:{:.2}mg, z:{:.2}mg",
        acc_0[0], acc_0[1], acc_0[2]
    );

    let acc_1 = acc_mpu_1.get_acc().await.unwrap();
    info!(
        "Accelerometer 1 init done! Current x:{:.2}mg, y:{:.2}mg, z:{:.2}mg",
        acc_1[0], acc_1[1], acc_1[2]
    );

    log::warn!("Going to calibrate Accelerometers hold them steady with z axis pointing up for 5s");

    let offsets_0 = calibrate_accel(&mut acc_mpu_0).await;

    info!(
        "Accelerometer 0 calibrated! Current offset_x:{:.2}mg, offset_y:{:.2}mg, offset_z:{:.2}mg",
        offsets_0[0], offsets_0[1], offsets_0[2]
    );

    let offsets_1 = calibrate_accel(&mut acc_mpu_1).await;

    info!(
        "Accelerometer 1 calibrated! Current offset_x:{:.2}mg, offset_y:{:.2}mg, offset_z:{:.2}mg",
        offsets_1[0], offsets_1[1], offsets_1[2]
    );

    // GPIO21 -> gyro reset, GPIO02 -> batt level
    set_interrupts(peripherals.GPIO21, peripherals.GPIO2);

    // Right arm -> Bservo0 -> Sservo 0 -> Sservo1
    let bservo_r_0 = peripherals.GPIO10;
    let sservo_r_0 = peripherals.GPIO20;
    let sservo_r_1 = peripherals.GPIO8;

    // Left arm -> Bservo1 -> Sservo 2 -> Sservo3
    let bservo_l_1 = peripherals.GPIO3;
    let sservo_l_2 = peripherals.GPIO4;
    let sservo_l_3 = peripherals.GPIO5;

    let mut ledc = Ledc::new(peripherals.LEDC);
    ledc.set_global_slow_clock(esp_hal::ledc::LSGlobalClkSource::APBClk);

    // This timer can be shared by all servos, its fine, servos just take measurments from this
    let mut servo_timer = ledc.timer::<LowSpeed>(timer::Number::Timer0);
    servo_timer
        .configure(timer::config::Config {
            duty: timer::config::Duty::Duty12Bit,
            clock_source: timer::LSClockSource::APBClk,
            frequency: Rate::from_hz(50),
        })
        .unwrap();

    let mut bservo_r_0 = make_servo(bservo_r_0, &servo_timer, Number::Channel0, &mut ledc);
    let mut sservo_r_0 = make_servo(sservo_r_0, &servo_timer, Number::Channel1, &mut ledc);
    let mut sservo_r_1 = make_servo(sservo_r_1, &servo_timer, Number::Channel2, &mut ledc);
    let mut bservo_l_1 = make_servo(bservo_l_1, &servo_timer, Number::Channel3, &mut ledc);
    let mut sservo_l_2 = make_servo(sservo_l_2, &servo_timer, Number::Channel4, &mut ledc);
    let mut sservo_l_3 = make_servo(sservo_l_3, &servo_timer, Number::Channel5, &mut ledc);

    let starting_angles = [0u8, 0u8, 90u8];

    let mut hand_left_curr_pos = [0.0, 0.0, 0.0];
    let mut hand_left_last_servos = starting_angles.clone(); // elbow 90 degrees
    let mut hand_right_curr_pos = [0.0, 0.0, 0.0];
    let mut hand_right_last_servos = starting_angles.clone(); // elbow 90 degrees

    let mut ticker = embassy_time::Ticker::every(Duration::from_millis(125));
    let mut last = Instant::now();

    let mut loop_i = 0u32;

    loop {
        loop_i += 1;
        ticker.next().await;

        let to_reset = critical_section::with(|cs| *TO_GYRO_RESET.borrow_ref_mut(cs));

        if to_reset {
            log::warn!(
                "Going to calibrate Accelerometers hold them steady with z axis pointing up for 5s"
            );
            let offsets_0 = calibrate_accel(&mut acc_mpu_0).await;
            info!(
                "Acc 0 calibrated! Offset_x:{:.2}mg, offset_y:{:.2}mg, offset_z:{:.2}mg",
                offsets_0[0], offsets_0[1], offsets_0[2]
            );
            let offsets_1 = calibrate_accel(&mut acc_mpu_1).await;
            info!(
                "Acc 1 calibrated! Offset_x:{:.2}mg, offset_y:{:.2}mg, offset_z:{:.2}mg",
                offsets_1[0], offsets_1[1], offsets_1[2]
            );
            hand_left_curr_pos = [0.0f32, 0.0f32, 0.0f32];
            hand_right_curr_pos = [0.0f32, 0.0f32, 0.0f32];

            angle_on_servo(&mut bservo_r_0, starting_angles[0]);
            angle_on_servo(&mut sservo_r_0, starting_angles[1]);
            angle_on_servo(&mut sservo_r_1, starting_angles[2]);
            angle_on_servo(&mut bservo_l_1, starting_angles[0]);
            angle_on_servo(&mut sservo_l_2, starting_angles[1]);
            angle_on_servo(&mut sservo_l_3, starting_angles[2]);

            hand_right_last_servos = starting_angles;
            hand_left_last_servos = starting_angles;

            critical_section::with(|cs| {
                *TO_GYRO_RESET.borrow_ref_mut(cs) = false;
            });
            continue;
        }

        let acc_0 = acc_mpu_0.get_acc().await.unwrap();
        let (x0, y0, z0) = (
            acc_0.x - offsets_0[0],
            acc_0.y - offsets_0[1],
            acc_0.z - offsets_0[2],
        );

        let acc_1 = acc_mpu_1.get_acc().await.unwrap();
        let (x1, y1, z1) = (
            acc_1.x - offsets_1[0],
            acc_1.y - offsets_1[1],
            acc_1.z - offsets_1[2],
        );

        let now = Instant::now();
        let dt_s = (now - last).as_micros() as f32 / 1_000_000.0;
        last = now;

        hand_right_curr_pos = [
            hand_right_curr_pos[0] + x0 * dt_s * dt_s,
            hand_right_curr_pos[1] + y0 * dt_s * dt_s,
            hand_right_curr_pos[2] + z0 * dt_s * dt_s,
        ];

        hand_left_curr_pos = [
            hand_left_curr_pos[0] + x1 * dt_s * dt_s,
            hand_left_curr_pos[1] + y1 * dt_s * dt_s,
            hand_left_curr_pos[2] + z1 * dt_s * dt_s,
        ];

        if loop_i % 100 == 0 {
            info!("Right hand:{hand_right_curr_pos:?}, left hand: {hand_left_curr_pos:?}");
        }

        let elbow = [0u8, 90u8, 5u8];
        let upper_arm = [0u8, 250u8, 5u8];
        let shoulder = [0u8, 250u8, 5u8];

        let left_hand_new_angles = angles_from_pos(
            hand_left_last_servos,
            hand_left_curr_pos,
            shoulder,
            upper_arm,
            elbow,
        );

        let right_hand_new_angles = angles_from_pos(
            hand_right_last_servos,
            hand_right_curr_pos,
            shoulder,
            upper_arm,
            elbow,
        );

        esp_println::println!(
            "Right Servos last:{hand_right_last_servos:?} | Right H Pos:{hand_right_curr_pos:?} | Right H New Servos: {right_hand_new_angles:?} | Dt:{dt_s:.3}s | Right H Acc: {acc_0}"
        );

        esp_println::println!(
            "Left Servos last:{hand_left_last_servos:?} | Left H Pos:{hand_left_curr_pos:?} | Left H New Servos: {left_hand_new_angles:?} | Dt:{dt_s:.3}s | Left H Acc: {acc_1}"
        );

        angle_on_servo(&mut bservo_r_0, right_hand_new_angles[0]);
        angle_on_servo(&mut sservo_r_0, right_hand_new_angles[1]);
        angle_on_servo(&mut sservo_r_1, right_hand_new_angles[2]);
        angle_on_servo(&mut bservo_l_1, left_hand_new_angles[0]);
        angle_on_servo(&mut sservo_l_2, left_hand_new_angles[1]);
        angle_on_servo(&mut sservo_l_3, left_hand_new_angles[2]);

        hand_right_last_servos = right_hand_new_angles;
        hand_left_last_servos = left_hand_new_angles;
    }
    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v1.2.2/examples
}

#[esp_hal::handler]
// #[esp_hal::ram]
fn handler() {
    esp_println::println!("GPIO Interrupt");
    if critical_section::with(|cs| {
        BUTTON_GYRO_RESET
            .borrow_ref_mut(cs)
            .as_mut()
            .unwrap()
            .is_interrupt_set()
    }) {
        esp_println::println!("Gyro Reset was the source of the interrupt");
        critical_section::with(|cs| {
            *TO_GYRO_RESET.borrow_ref_mut(cs) = true;

            // If you dont reset then it will just loop the interrupt handler
            BUTTON_GYRO_RESET
                .borrow_ref_mut(cs)
                .as_mut()
                .unwrap()
                .clear_interrupt()
        });
    } else {
        esp_println::println!("Battery Read was the source of the interrupt");
    }
}
