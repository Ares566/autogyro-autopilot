//! Конфигурация аппаратного обеспечения автопилота

use embassy_rp::peripherals::*;
use embassy_rp::gpio::Pin;

/// Helper macro to create interrupt executors.
macro_rules! interrupt_executor {
    ($interrupt:ident, $prio:ident) => {{
        use embassy_executor::InterruptExecutor;
        use embassy_rp::interrupt;
        use embassy_rp::interrupt::{InterruptExt, Priority};

        interrupt::$interrupt.set_priority(Priority::$prio);
        static EXECUTOR: InterruptExecutor = InterruptExecutor::new();
        let spawner = EXECUTOR.start(interrupt::$interrupt);

        #[interrupt]
        #[allow(non_snake_case)]
        unsafe fn $interrupt() {
            EXECUTOR.on_interrupt()
        }

        spawner
    }};
}

/// Конфигурация пинов GPIO
pub mod pins {
    use super::*;

    /// I2C для датчиков (IMU, барометр)
    pub mod i2c {
        use super::*;

        /// Пин SDA для I2C0
        pub const SDA_PIN: u8 = 4;  // GPIO4
        /// Пин SCL для I2C0
        pub const SCL_PIN: u8 = 5;  // GPIO5
    }

    /// UART для GPS
    pub mod gps {
        use super::*;

        /// Пин TX для UART0
        pub const TX_PIN: u8 = 0;   // GPIO0
        /// Пин RX для UART0
        pub const RX_PIN: u8 = 1;   // GPIO1
    }

    /// UART для телеметрии
    pub mod telemetry {
        use super::*;

        /// Пин TX для UART1
        pub const TX_PIN: u8 = 8;   // GPIO8
        /// Пин RX для UART1
        pub const RX_PIN: u8 = 9;   // GPIO9
    }

    /// PWM для сервоприводов
    pub mod servos {
        use super::*;

        /// Пин для серво циклического шага по тангажу
        pub const PITCH_PIN: u8 = 10;  // GPIO10 - PWM5A
        /// Пин для серво циклического шага по крену
        pub const ROLL_PIN: u8 = 12;   // GPIO11 - PWM5B
    }

    /// DShot для ESC моторов
    pub mod motors {
        use super::*;

        /// Пин для левого мотора
        pub const LEFT_PIN: u8 = 11;   // GPIO12
        /// Пин для правого мотора
        pub const RIGHT_PIN: u8 = 13;  // GPIO13
    }

    /// Дополнительные пины
    pub mod misc {
        use super::*;

        /// Встроенный светодиод на Pico
        pub const LED_PIN: u8 = 25;    // GPIO25

        /// Пин для зуммера (опционально)
        pub const BUZZER_PIN: u8 = 14; // GPIO14

        /// Пин для кнопки арминга (опционально)
        pub const ARM_BUTTON_PIN: u8 = 15; // GPIO15

        /// Пин для датчика оборотов ротора (опционально)
        pub const RPM_SENSOR_PIN: u8 = 16; // GPIO16
    }
}

/// Конфигурация частот и скоростей
pub mod frequencies {
    /// Частота I2C шины (Гц)
    pub const I2C_FREQUENCY: u32 = 400_000; // 400 kHz

    /// Скорость UART для GPS (бод)
    pub const GPS_BAUDRATE: u32 = 9_600;   // Стандарт для большинства GPS

    /// Скорость UART для телеметрии (бод)
    pub const TELEMETRY_BAUDRATE: u32 = 115_200;

    /// Частота PWM для сервоприводов (Гц)
    pub const SERVO_PWM_FREQUENCY: u32 = 50; // 50 Hz стандарт для сервоприводов

    /// Частота DShot для ESC
    pub const DSHOT_FREQUENCY: u32 = 600_000; // DShot600
}

/// Конфигурация PWM для сервоприводов
pub mod servo {
    /// Минимальная длительность импульса (мкс)
    pub const MIN_PULSE_US: u16 = 1000;

    /// Центральная длительность импульса (мкс)
    pub const CENTER_PULSE_US: u16 = 1500;

    /// Максимальная длительность импульса (мкс)
    pub const MAX_PULSE_US: u16 = 2000;

    /// Период PWM сигнала (мкс)
    pub const PWM_PERIOD_US: u32 = 20_000; // 20ms = 50Hz

    /// Максимальный угол отклонения сервопривода (градусы)
    pub const MAX_ANGLE_DEG: f32 = 45.0;
}

/// Конфигурация DShot протокола
pub mod dshot {
    /// Тип DShot протокола
    pub const DSHOT_TYPE: DshotSpeed = DshotSpeed::DShot600;

    /// Минимальное значение газа
    pub const THROTTLE_MIN: u16 = 48;

    /// Максимальное значение газа  
    pub const THROTTLE_MAX: u16 = 2047;

    /// Значение для специальных команд
    pub const SPECIAL_COMMAND_THRESHOLD: u16 = 48;

    #[derive(Debug, Clone, Copy)]
    pub enum DshotSpeed {
        DShot150,
        DShot300,
        DShot600,
        DShot1200,
    }

    impl DshotSpeed {
        /// Получить длительность бита в наносекундах
        pub const fn bit_period_ns(&self) -> u32 {
            match self {
                DshotSpeed::DShot150 => 6667,  // 150kHz
                DshotSpeed::DShot300 => 3333,  // 300kHz
                DshotSpeed::DShot600 => 1667,  // 600kHz
                DshotSpeed::DShot1200 => 833,  // 1200kHz
            }
        }
    }
}

/// Адреса I2C устройств
pub mod i2c_addresses {
    /// Адрес MPU6050 IMU
    pub const MPU6050_ADDR: u8 = 0x68;

    /// Альтернативный адрес MPU6050 (если AD0 = HIGH)
    pub const MPU6050_ADDR_ALT: u8 = 0x69;

    /// Адрес BMP280 барометра
    pub const BMP280_ADDR: u8 = 0x76;

    /// Альтернативный адрес BMP280 (если SDO = HIGH)
    pub const BMP280_ADDR_ALT: u8 = 0x77;

    /// Адрес магнитометра HMC5883L
    pub const HMC5883L_ADDR: u8 = 0x1E;

    /// Адрес магнитометра QMC5883L
    pub const QMC5883L_ADDR: u8 = 0x0D;
}

/// Параметры системы
pub mod system {
    /// Частота главного цикла управления (Гц)
    pub const CONTROL_LOOP_RATE_HZ: u32 = 50;

    /// Частота опроса IMU (Гц)
    pub const IMU_SAMPLE_RATE_HZ: u32 = 100;

    /// Частота опроса барометра (Гц)
    pub const BARO_SAMPLE_RATE_HZ: u32 = 25;

    /// Частота обновления GPS (Гц)
    pub const GPS_UPDATE_RATE_HZ: u32 = 5;

    /// Частота отправки телеметрии (Гц)
    pub const TELEMETRY_RATE_HZ: u32 = 10;

    /// Таймаут для критических операций (мс)
    pub const CRITICAL_TIMEOUT_MS: u64 = 1000;

    /// Количество попыток инициализации устройств
    pub const INIT_RETRY_COUNT: u8 = 3;
}

/// Лимиты безопасности
pub mod safety {
    /// Максимальный угол крена (градусы)
    pub const MAX_ROLL_ANGLE_DEG: f32 = 30.0;

    /// Максимальный угол тангажа (градусы)
    pub const MAX_PITCH_ANGLE_DEG: f32 = 25.0;

    /// Максимальная угловая скорость (градусы/сек)
    pub const MAX_ANGULAR_RATE_DEG_S: f32 = 180.0;

    /// Минимальная высота для автоматических маневров (метры)
    pub const MIN_AUTO_ALTITUDE_M: f32 = 5.0;

    /// Максимальная высота полета (метры)
    pub const MAX_ALTITUDE_M: f32 = 120.0;

    /// Минимальное напряжение батареи (вольты)
    pub const MIN_BATTERY_VOLTAGE: f32 = 10.5;

    /// Критическое напряжение батареи (вольты)
    pub const CRITICAL_BATTERY_VOLTAGE: f32 = 10.0;

    /// Максимальная скорость снижения (м/с)
    pub const MAX_DESCENT_RATE_MS: f32 = 3.0;

    /// Максимальная скорость подъема (м/с)
    pub const MAX_CLIMB_RATE_MS: f32 = 5.0;
}

/// Параметры фильтрации
pub mod filters {
    /// Коэффициент комплементарного фильтра для IMU (0.0 - 1.0)
    pub const COMPLEMENTARY_FILTER_ALPHA: f32 = 0.98;

    /// Частота среза фильтра нижних частот для акселерометра (Гц)
    pub const ACCEL_LPF_CUTOFF_HZ: f32 = 5.0;

    /// Частота среза фильтра нижних частот для барометра (Гц)
    pub const BARO_LPF_CUTOFF_HZ: f32 = 1.0;

    /// Размер скользящего среднего для GPS
    pub const GPS_MOVING_AVERAGE_SIZE: usize = 5;
}
