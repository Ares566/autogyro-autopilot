
#![no_std]
#![no_main]



use embassy_executor::Spawner;
use embassy_rp::{bind_interrupts, peripherals};
use embassy_rp::gpio::{Level, Output};
use embassy_rp::i2c::{self, Config as I2cConfig};
use embassy_rp::peripherals::{I2C0, PIN_0, PIN_1, PIN_2, PIN_3, PIN_4, PIN_5, PIN_6, PIN_7, UART0};
use embassy_rp::pwm::{self, Pwm};
use embassy_rp::uart::{self, Config as UartConfig};
use embassy_time::{Duration, Timer};
use {defmt_rtt as _, panic_probe as _};

mod config;
mod data;
mod drivers;
// mod sensors;
// mod control;
// mod flight;
mod tasks;
mod control;
mod utils;
//mod utils;


use crate::config::hardware::*;
use crate::data::{SYSTEM_STATE, FlightMode};
use crate::drivers::actuators::dshot::DshotPio;
use crate::drivers::actuators::servo::Servo;
use crate::tasks::*;
use utils::system_info;
use crate::utils::system_info::get_system_clocks;
use nalgebra::*;

/// Точка входа в программу
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // Инициализация HAL Raspberry Pi Pico
    let p = embassy_rp::init(Default::default());

    defmt::info!("=== Автопилот автожира v0.1.0 ===");
    defmt::info!("Инициализация системы...");
    // Вывод информации о частотах
    system_info::print_clock_info();

    // Проверка корректности частот
    if let Err(e) = system_info::validate_clocks() {
        defmt::error!("Ошибка конфигурации частот: {}", e);
        panic!("Invalid clock configuration");
    }
    // Настройка светодиода для индикации состояния
    let mut led = Output::new(p.PIN_25, Level::Low);

    // Мигаем светодиодом при старте
    for _ in 0..3 {
        led.set_high();
        Timer::after(Duration::from_millis(100)).await;
        led.set_low();
        Timer::after(Duration::from_millis(100)).await;
    }

    // Инициализация I2C для датчиков (IMU и барометр)
    let i2c = {
        let sda = p.PIN_4; // GPIO4 - SDA
        let scl = p.PIN_5; // GPIO5 - SCL

        let mut config = I2cConfig::default();
        config.frequency = 400_000; // 400 kHz для быстрого обмена

        i2c::I2c::new_blocking(p.I2C0, scl, sda, config)
    };

    // Инициализация UART для GPS
    let uart_gps = {
        let tx = p.PIN_0; // GPIO0 - TX
        let rx = p.PIN_1; // GPIO1 - RX

        let mut config = UartConfig::default();
        config.baudrate = 9600; // Стандартная скорость для GPS

        uart::Uart::new_blocking(p.UART0, tx, rx, config)
    };

    // Инициализация UART для телеметрии/отладки
    let uart_telem = {
        let tx = p.PIN_8; // GPIO8 - TX
        let rx = p.PIN_9; // GPIO9 - RX

        let mut config = UartConfig::default();
        config.baudrate = 115200; // Высокая скорость для телеметрии

        uart::Uart::new_blocking(p.UART1, tx, rx, config)
    };

    // TODO Инициализацию в actuators::servo
    // Инициализация PWM для сервоприводов
    let pwm_servo_pitch = {
        // let pin = p.PIN_10; // GPIO10 - Серво тангажа
        // let mut pwm = Pwm::new_output_a(p.PWM_CH5, pin, Default::default());
        // 
        // // Настройка для сервоприводов: 50Hz (20ms период)
        // let mut cfg = pwm::Config::default();
        // cfg.divider = 125.into(); // 125MHz / 125 = 1MHz
        // cfg.top = 20000; // 1MHz / 20000 = 50Hz
        // pwm.set_config(&cfg);
        // 
        // pwm
        Servo::new().unwrap()
    };

    let pwm_servo_roll = {
        // let pin = p.PIN_11; // GPIO11 - Серво крена
        // let mut pwm = Pwm::new_output_b(p.PWM_CH5, pin, Default::default());
        // 
        // // Используем тот же таймер PWM_CH5
        // let mut cfg = pwm::Config::default();
        // cfg.divider = 125.into();
        // cfg.top = 20000;
        // pwm.set_config(&cfg);
        // 
        // pwm
        Servo::new().unwrap()
    };

    // Инициализация DShot для ESC моторов
    let _motor_left_pin = p.PIN_11;  // GPIO12 - Левый мотор
    let _motor_right_pin = p.PIN_12; // GPIO13 - Правый мотор
    bind_interrupts!( struct Pio0Irqs {
        PIO0_IRQ_0 => embassy_rp::pio::InterruptHandler<peripherals::PIO0>;
    });

    let clocks = get_system_clocks();
    let divider = (clocks.sys_freq as f32) / (8.0 * 300.0 * 1000.0); // = 26.0 для 125MHz DShot 600
    let dshot = DshotPio::<2, _>::new(
        p.PIO0,
        Pio0Irqs,
        _motor_left_pin,
        _motor_right_pin,
        (divider as u16,0)
        //(52, 0),  // clock divider
    );
    //defmt::info!("DShot clock divider {}", divider as u16);

    // Проверка критических компонентов перед запуском
    defmt::info!("Проверка подсистем...");

    // Устанавливаем начальный режим полета
    *SYSTEM_STATE.flight_mode.lock().await = FlightMode::Disarmed;

    // Запуск асинхронных задач
    defmt::info!("Запуск задач...");

    // Задача опроса датчиков
    spawner.spawn(sensor_task::task(i2c)).unwrap();

    // Задача обработки GPS
    //spawner.spawn(tasks::gps_task(uart_gps)).unwrap();

    // TODO Задача управления полетом
    spawner.spawn(control_task::task()).unwrap();

    // TODO вынести на отдельное ядро
    // Задача управления исполнительными механизмами
    spawner.spawn(actuator_task::task(
        dshot,
        pwm_servo_pitch,
        pwm_servo_roll,
    )).unwrap();

    // Задача телеметрии
    //spawner.spawn(tasks::telemetry_task(uart_telem)).unwrap();

    // Задача мониторинга состояния системы
    //spawner.spawn(tasks::monitor_task(led)).unwrap();

    defmt::info!("Система инициализирована. Ожидание готовности датчиков...");

    // Ждем готовности всех подсистем
    loop {
        if SYSTEM_STATE.is_ready_for_flight().await {
            defmt::info!("Все подсистемы готовы к работе!");
            break;
        }
        Timer::after(Duration::from_millis(500)).await;
    }

    // Основной цикл для обработки команд высокого уровня
    loop {
        // Проверяем режим полета и выполняем соответствующие действия
        let mode = *SYSTEM_STATE.flight_mode.lock().await;

        match mode {
            FlightMode::Disarmed => {
                // Ждем команды на взлет
                // TODO: Добавить обработку команд с наземной станции
            }
            FlightMode::TakeOff => {
                defmt::info!("Инициирован автоматический взлет");
                // Логика взлета обрабатывается в control_task
            }
            FlightMode::Landing => {
                defmt::info!("Инициирована автоматическая посадка");
                // Логика посадки обрабатывается в control_task
            }
            FlightMode::Emergency => {
                defmt::error!("АВАРИЙНЫЙ РЕЖИМ! Экстренная посадка!");
                // Переключаем все системы в безопасный режим
                SYSTEM_STATE.armed.store(false, core::sync::atomic::Ordering::Relaxed);
            }
            _ => {}
        }

        // Главный цикл выполняется с низкой частотой
        Timer::after(Duration::from_millis(100)).await;
    }
}
