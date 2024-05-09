#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

// mod motor;

use defmt::*; // if info!() or other debug macros are used
use embassy_executor::Spawner;
use embassy_time::{Duration, Instant, Timer};
use embassy_stm32::gpio::{AnyPin, Input, Level, Output, OutputType, Pin, Pull, Speed};
use embassy_stm32::time::khz;
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
use embassy_stm32::timer::{Channel, OutputPolarity};
use embassy_stm32::usart::{Config, Uart};
use embassy_stm32::{bind_interrupts, Peripheral, peripherals, Peripherals, usart};
use embassy_stm32::peripherals::TIM1;
use heapless::String;
use core::fmt::Write;
use regex_automata::meta::Regex;

//noinspection RsUnusedImport
use {defmt_rtt as _, panic_probe as _};

use talc::*;

static mut ARENA: [u8; 10000] = [0; 10000];

#[global_allocator]
static ALLOCATOR: Talck<spin::Mutex<()>, ClaimOnOom> = Talc::new(unsafe {
    // if we're in a hosted environment, the Rust runtime may allocate before
    // main() is called, so we need to initialize the arena automatically
    ClaimOnOom::new(Span::from_const_array(core::ptr::addr_of!(ARENA)))
}).lock();




use core::sync::atomic::Ordering;
use atomic_float::AtomicF32;
static LED_SPEED_SHARE: AtomicF32 = AtomicF32::new(0.0);

use core::sync::atomic::AtomicI16;
use embassy_stm32::exti::{AnyChannel, ExtiInput};
use embassy_stm32::exti::Channel as ExtiChannel; // to avoid confusion with the PWM Channel

static DUTY_A: AtomicI16 = AtomicI16::new(0);
static DUTY_B: AtomicI16 = AtomicI16::new(0);

use core::sync::atomic::AtomicU32;
static DURATION_A: AtomicU32 = AtomicU32::new(0);
static DURATION_B: AtomicU32 = AtomicU32::new(0);


bind_interrupts!(struct Irqs {
    USART1 => usart::InterruptHandler<peripherals::USART1>;
});

#[embassy_executor::main]
/// Main function, blinks an LED for 200ms on, 300ms off, and prints the current loop number to the console.
async fn main(spawner: Spawner) {
    // Hardware objects
    let p = embassy_stm32::init(Default::default());

    let ch1 = PwmPin::new_ch1(p.PA8, OutputType::PushPull);
    let ch2 = PwmPin::new_ch2(p.PA9, OutputType::PushPull);
    let ch3 = PwmPin::new_ch3(p.PA10, OutputType::PushPull);
    let ch4 = PwmPin::new_ch4(p.PA11, OutputType::PushPull);

    let mut usart = Uart::new(p.USART1, p.PB7, p.PB6, Irqs, p.DMA2_CH7, p.DMA2_CH2, Config::default(),).unwrap();

    let mut pwm_a = SimplePwm::new(p.TIM1, Some(ch1), Some(ch2), None, None, khz(50), Default::default());
    // have to create a second instance of the timer to use the other channels with a separate pwm variable
    // this means using the .steal() method to create a second TIM1 instance out of thin air
    // which is an unsafe function but is safe to use in this context, as both PWM instances have the same parameters
    let mut pwm_b = SimplePwm::new(unsafe {TIM1::steal()}, None, None, Some(ch3), Some(ch4), khz(50), Default::default());


    // Variables
    // let mut duty: i16 = 50;
    let mut s: String<128> = String::new();

    // Functions
    pwm_a.set_polarity(Channel::Ch1, OutputPolarity::ActiveLow); // inverted PWM
    pwm_a.set_polarity(Channel::Ch2, OutputPolarity::ActiveLow);
    pwm_b.set_polarity(Channel::Ch3, OutputPolarity::ActiveLow);
    pwm_b.set_polarity(Channel::Ch4, OutputPolarity::ActiveLow);


    info!("Starting Program...");
    core::writeln!(&mut s, "Starting Program...\r").unwrap();
    unwrap!(usart.write(s.as_bytes()).await);
    s.clear();
    
    spawner.spawn(blinky(p.PC13.degrade())).unwrap();
    spawner.spawn(remote_ch_1(p.PA2.degrade(), p.EXTI2.degrade(), 'a')).unwrap();
    Timer::after_millis(50).await;
    spawner.spawn(remote_ch_2(p.PA3.degrade(), p.EXTI3.degrade(), 'b')).unwrap();
    Timer::after_millis(10).await;
    spawner.spawn(print_durations()).unwrap();
    
    enable_motor(&mut pwm_a, 'a');
    enable_motor(&mut pwm_b, 'b');



    // UART Handling

    let re = Regex::new(r"M[1|2][0-9a-fA-f]{2}").unwrap();

    let mut current_char: [u8; 1] = [0; 1];
    let mut msg: [u8; 4] = [0; 4];
    let mut index = 0usize;
    loop {
        /*unwrap!(usart.read(&mut current_char).await);
        if index > 2 {
            index = 3;
        }
        if (current_char[0] == 13) | (current_char[0] == 10) {
            // carriage return or line feed
            if index == 3 {
                let command = core::str::from_utf8(&msg).unwrap();
                if re.is_match(command) {
                    let motor = command.chars().nth(1).unwrap();
                    let duty = ((u8::from_str_radix(&command[2..], 16).unwrap() as i8) as i16)*100/128;
                    if motor == '1' {
                        set_motor_duty(&mut pwm_a, 'a', duty);
                    } else if motor == '2' {
                        set_motor_duty(&mut pwm_b, 'b', duty);
                    }
                    unwrap!(usart.write("\r\n".as_ref()).await);
                    core::writeln!(&mut s, "Motor {} set to {}%\r", motor, duty).unwrap();
                    unwrap!(usart.write(s.as_bytes()).await);
                    s.clear();
                    msg = [0; 4];
                    index = 0;
                } else {
                    unwrap!(usart.write("\r\n".as_ref()).await);
                    core::writeln!(&mut s, "Invalid Command\r").unwrap();
                    unwrap!(usart.write(s.as_bytes()).await);
                    s.clear();
                    msg = [0; 4]; // Clear the current input
                    index = 0;
                }
            }
        } else if (current_char[0] == 8) | (current_char[0] == 127) {
            // backspace
            if index > 0 {
                for n in index..msg.len() {
                    msg[n] = 0;
                }
                // msg[index] = 0;
                info!("Index: {}", index);
                if index < 4 {
                    index -= 1;
                }
                unwrap!(usart.write("\r    \r".as_ref()).await);
                unwrap!(usart.write(&msg).await);
            }
        } else {
            msg[index] = current_char[0];
            // info!("Current char: {}", current_char[0]);
            index += 1;
            unwrap!(usart.write("\r".as_ref()).await);
            // unwrap!(usart.write(&backspace).await);
            unwrap!(usart.write(&msg).await);
        }*/
        let duration_a = DURATION_A.load(Ordering::Acquire);
        let duration_b = DURATION_B.load(Ordering::Acquire);

        // Motor A
        if !(900..=2100).contains(&duration_a) {
            DUTY_A.store(0, Ordering::Release);
        } else if duration_a > 1_950 {
            DUTY_A.store(100, Ordering::Release);
        } else if duration_a < 1_050 {
            DUTY_A.store(-100, Ordering::Release);
        } else if (1_450 < duration_a) && (duration_a < 1_550) {
            DUTY_A.store(0, Ordering::Release);
        } else if duration_a < 1_425 {
            DUTY_A.store(-(100-((duration_a - 1000)/5) as i16), Ordering::Release);
        } else if duration_a > 1_575 {
            DUTY_A.store((100 - ((2000 - duration_a) / 5)) as i16, Ordering::Release);
        } else {
            DUTY_A.store(0, Ordering::Release);
        }

        // Motor B
        if !(900..=2100).contains(&duration_b) {
            DUTY_B.store(0, Ordering::Release);
        } else if duration_b > 1_950 {
            DUTY_B.store(100, Ordering::Release);
        } else if duration_b < 1_050 {
            DUTY_B.store(-100, Ordering::Release);
        } else if (1_450 < duration_b) && (duration_b < 1_550) {
            DUTY_B.store(0, Ordering::Release);
        } else if duration_b < 1_425 {
            DUTY_B.store(-(100-((duration_b - 1000)/5) as i16), Ordering::Release);
        } else if duration_b > 1_575 {
            DUTY_B.store((100-((2000-duration_b)/5)) as i16, Ordering::Release);
        } else {
            DUTY_B.store(0, Ordering::Release);
        }

        set_motor_duty(&mut pwm_a, 'a', DUTY_A.load(Ordering::Acquire));
        set_motor_duty(&mut pwm_b, 'b', DUTY_B.load(Ordering::Acquire));
        Timer::after_millis(50).await;
    }
}

#[embassy_executor::task]
async fn print_durations() {
    loop {
        info!("Duration A: {} us", DURATION_A.load(Ordering::Acquire));
        info!("Duration B: {} us", DURATION_B.load(Ordering::Acquire));
        Timer::after_millis(200).await;
    }
}

#[embassy_executor::task]
async fn remote_ch_1(pin: AnyPin, exti_ch: AnyChannel, motor: char) {
    let pin = Input::new(pin, Pull::None);
    let mut pin = ExtiInput::new(pin, exti_ch);
    loop {
        pin.wait_for_rising_edge().await;
        let inst = Instant::now();
        pin.wait_for_falling_edge().await;
        let duration = Instant::checked_duration_since(&Instant::now(), inst).unwrap().as_micros();
        if motor == 'a' {
            DURATION_A.store(duration as u32, Ordering::Release);
        } else {
            DURATION_B.store(duration as u32, Ordering::Release);
        }
    }
}

#[embassy_executor::task]
async fn remote_ch_2(pin: AnyPin, exti_ch: AnyChannel, motor: char) {
    let pin = Input::new(pin, Pull::None);
    let mut pin = ExtiInput::new(pin, exti_ch);
    loop {
        pin.wait_for_rising_edge().await;
        let inst = Instant::now();
        pin.wait_for_falling_edge().await;
        let duration = Instant::checked_duration_since(&Instant::now(), inst).unwrap().as_micros();
        if motor == 'a' {
            DURATION_A.store(duration as u32, Ordering::Release);
        } else {
            DURATION_B.store(duration as u32, Ordering::Release);
        }
    }
}

#[embassy_executor::task]
async fn blinky(pin: AnyPin) {
    let mut led = Output::new(pin, Level::Low, Speed::Low);

    loop {
        let speed = LED_SPEED_SHARE.load(Ordering::Acquire); // Get the current speed from the shared resource
        // info!("Speed: {}", speed);
        match speed {
            0.0 => {
                led.set_high();
                Timer::after_millis(20).await; // don't run this too fast
            },
            _ => {
                led.set_low();
                Timer::after_millis((5000.0/speed) as u64).await;
                led.set_high();
                Timer::after_millis((5000.0/speed) as u64).await;
            }
        }
    }
}


// TODO: Put these in the motor.rs file
fn enable_motor(pwm: &mut SimplePwm<TIM1>, motor: char) {
    let channels = match motor {
        'a' => [Channel::Ch1, Channel::Ch2],
        'b' => [Channel::Ch3, Channel::Ch4],
        _ => [Channel::Ch1, Channel::Ch2], // TODO: Return an error instead of assuming motor a by default
    };

    pwm.enable(channels[0]);
    pwm.enable(channels[1]);
}

fn disable_motor(pwm: &mut SimplePwm<TIM1>, motor: char) {
    let channels = match motor {
        'a' => [Channel::Ch1, Channel::Ch2],
        'b' => [Channel::Ch3, Channel::Ch4],
        _ => [Channel::Ch1, Channel::Ch2], // TODO: Return an error instead of assuming motor a by default
    };

    pwm.disable(channels[0]);
    pwm.disable(channels[1]);
    set_motor_duty(pwm, motor, 0);
}

fn set_motor_duty(pwm: &mut SimplePwm<TIM1>, motor: char, duty: i16) {
    let clamped_duty = duty.clamp(-100, 100) as i32;
    let max = pwm.get_max_duty() as u32;

    let channels = match motor {
        'a' => [Channel::Ch1, Channel::Ch2],
        'b' => [Channel::Ch3, Channel::Ch4],
        _ => [Channel::Ch1, Channel::Ch2], // TODO: Return an error instead of assuming motor a by default
    };

    // info!("Motor: {}", motor);
    // info!("Clamped Duty: {}", clamped_duty);

    if duty == 0 {
        pwm.set_duty(channels[0], 0);
        pwm.set_duty(channels[1], 0);
    } else if duty <= 0 {
        pwm.set_duty(channels[0], 0);
        pwm.set_duty(channels[1], (clamped_duty.unsigned_abs()*max/100) as u16);
    } else {
        pwm.set_duty(channels[0], (clamped_duty.unsigned_abs()*max/100) as u16);
        pwm.set_duty(channels[1], 0);
    }
    match clamped_duty.unsigned_abs() {
        0 => LED_SPEED_SHARE.store(0.0, Ordering::Release), // Update the LED with the current speed
        // _ => LED_SPEED_SHARE.store(10.0 / (clamped_duty.unsigned_abs() as f32), Ordering::Release), // Update the LED with the current speed
        _ => LED_SPEED_SHARE.store(clamped_duty.unsigned_abs() as f32, Ordering::Release), // Update the LED with the current speed
    }
}