use cortex_m::prelude::{_embedded_hal_blocking_i2c_Write, _embedded_hal_blocking_i2c_WriteRead};
use defmt::{Format, Formatter};
use embassy_rp::i2c::{I2c, Error as I2cError, Blocking};

use embassy_rp::peripherals::I2C0;
use embassy_time::{Duration, Timer};

/// Адрес MPU6050 на шине I2C (по умолчанию)
const MPU6050_ADDR: u8 = 0x68;

/// Регистры MPU6050
#[allow(dead_code)]
mod regs {
    pub const PWR_MGMT_1: u8 = 0x6B;      // Управление питанием
    pub const PWR_MGMT_2: u8 = 0x6C;      // Управление питанием 2
    pub const CONFIG: u8 = 0x1A;           // Конфигурация
    pub const GYRO_CONFIG: u8 = 0x1B;     // Конфигурация гироскопа
    pub const ACCEL_CONFIG: u8 = 0x1C;    // Конфигурация акселерометра
    pub const SMPLRT_DIV: u8 = 0x19;      // Делитель частоты выборки
    pub const INT_ENABLE: u8 = 0x38;      // Включение прерываний
    pub const WHO_AM_I: u8 = 0x75;        // Идентификатор устройства

    // Регистры данных
    pub const ACCEL_XOUT_H: u8 = 0x3B;    // Старший байт X акселерометра
    pub const ACCEL_XOUT_L: u8 = 0x3C;    // Младший байт X акселерометра
    pub const ACCEL_YOUT_H: u8 = 0x3D;    // Старший байт Y акселерометра
    pub const ACCEL_YOUT_L: u8 = 0x3E;    // Младший байт Y акселерометра
    pub const ACCEL_ZOUT_H: u8 = 0x3F;    // Старший байт Z акселерометра
    pub const ACCEL_ZOUT_L: u8 = 0x40;    // Младший байт Z акселерометра
    pub const TEMP_OUT_H: u8 = 0x41;      // Старший байт температуры
    pub const TEMP_OUT_L: u8 = 0x42;      // Младший байт температуры
    pub const GYRO_XOUT_H: u8 = 0x43;     // Старший байт X гироскопа
    pub const GYRO_XOUT_L: u8 = 0x44;     // Младший байт X гироскопа
    pub const GYRO_YOUT_H: u8 = 0x45;     // Старший байт Y гироскопа
    pub const GYRO_YOUT_L: u8 = 0x46;     // Младший байт Y гироскопа
    pub const GYRO_ZOUT_H: u8 = 0x47;     // Старший байт Z гироскопа
    pub const GYRO_ZOUT_L: u8 = 0x48;     // Младший байт Z гироскопа
}

/// Диапазон измерения акселерометра
#[derive(Debug, Clone, Copy)]
pub enum AccelRange {
    /// ±2g
    G2 = 0x00,
    /// ±4g
    G4 = 0x08,
    /// ±8g
    G8 = 0x10,
    /// ±16g
    G16 = 0x18,
}

/// Диапазон измерения гироскопа
#[derive(Debug, Clone, Copy)]
pub enum GyroRange {
    /// ±250°/s
    Deg250 = 0x00,
    /// ±500°/s
    Deg500 = 0x08,
    /// ±1000°/s
    Deg1000 = 0x10,
    /// ±2000°/s
    Deg2000 = 0x18,
}

/// Ошибки работы с MPU6050
#[derive(Debug)]
pub enum Mpu6050Error {
    /// Ошибка I2C
    I2c(I2cError),
    /// Неверный идентификатор устройства
    InvalidDevice,
    /// Ошибка конфигурации
    ConfigError,
}

impl From<I2cError> for Mpu6050Error {
    fn from(error: I2cError) -> Self {
        Mpu6050Error::I2c(error)
    }
}

// Реализация Format для defmt
impl defmt::Format for Mpu6050Error {
    fn format(&self, fmt: defmt::Formatter) {
        match self {
            Mpu6050Error::I2c(e) => defmt::write!(fmt, "I2C error: {}", e),
            Mpu6050Error::InvalidDevice => defmt::write!(fmt, "Invalid device ID"),
            Mpu6050Error::ConfigError => defmt::write!(fmt, "Configuration error"),
        }
    }
}

/// Сырые данные с MPU6050
#[derive(Debug, Clone, Copy)]
pub struct RawData {
    /// Ускорение по X в единицах LSB
    pub accel_x_raw: i16,
    /// Ускорение по Y в единицах LSB
    pub accel_y_raw: i16,
    /// Ускорение по Z в единицах LSB
    pub accel_z_raw: i16,
    /// Угловая скорость по X в единицах LSB
    pub gyro_x_raw: i16,
    /// Угловая скорость по Y в единицах LSB
    pub gyro_y_raw: i16,
    /// Угловая скорость по Z в единицах LSB
    pub gyro_z_raw: i16,
    /// Температура в единицах LSB
    pub temp_raw: i16,
}

/// Обработанные данные с MPU6050
#[derive(Debug, Clone, Copy)]
pub struct ProcessedData {
    /// Ускорение по X в м/с²
    pub accel_x: f32,
    /// Ускорение по Y в м/с²
    pub accel_y: f32,
    /// Ускорение по Z в м/с²
    pub accel_z: f32,
    /// Угловая скорость по X в рад/с
    pub gyro_x: f32,
    /// Угловая скорость по Y в рад/с
    pub gyro_y: f32,
    /// Угловая скорость по Z в рад/с
    pub gyro_z: f32,
    /// Температура в °C
    pub temperature: f32,
}

pub struct Mpu6050 {
    /// Адрес устройства
    addr: u8,
    /// Диапазон акселерометра
    accel_range: AccelRange,
    /// Диапазон гироскопа
    gyro_range: GyroRange,
    /// Масштабный коэффициент для акселерометра
    accel_scale: f32,
    /// Масштабный коэффициент для гироскопа
    gyro_scale: f32,
}

impl Mpu6050{
    /// Создание нового экземпляра драйвера
    pub async fn new<I>(i2c: &mut I2c<'_, I, Blocking>) -> Result<Self, Mpu6050Error>
    where
        I: embassy_rp::i2c::Instance,
    {
        let mut mpu = Self {
            addr: MPU6050_ADDR,
            accel_range: AccelRange::G2,
            gyro_range: GyroRange::Deg250,
            accel_scale: 16384.0, // Для ±2g
            gyro_scale: 131.0,    // Для ±250°/s
        };

        // Инициализация устройства
        mpu.init(i2c).await?;

        Ok(mpu)
    }

    /// Инициализация MPU6050
    async fn init<I>(&mut self, i2c: &mut I2c<'_, I, Blocking>) -> Result<(), Mpu6050Error>
    where
        I: embassy_rp::i2c::Instance,
    {
        // Проверка идентификатора устройства
        let who_am_i = self.read_register(i2c, regs::WHO_AM_I).await?;
        if who_am_i != 0x68 && who_am_i != 0x72 {
            defmt::error!("Неверный ID устройства: 0x{:02x}", who_am_i);
            return Err(Mpu6050Error::InvalidDevice);
        }

        // Сброс устройства
        self.write_register(i2c, regs::PWR_MGMT_1, 0x80).await?;
        Timer::after(Duration::from_millis(100)).await;

        // Выход из режима сна, выбор источника тактирования (PLL с X гироскопом)
        self.write_register(i2c, regs::PWR_MGMT_1, 0x01).await?;
        Timer::after(Duration::from_millis(10)).await;

        // Конфигурация: DLPF = 3 (44Hz акселерометр, 42Hz гироскоп)
        self.write_register(i2c, regs::CONFIG, 0x03).await?;

        // Устанавливаем частоту выборки 100Hz: 1000Hz / (1 + 9) = 100Hz
        self.write_register(i2c, regs::SMPLRT_DIV, 9).await?;

        // Конфигурация акселерометра (по умолчанию ±2g)
        self.set_accel_range(i2c, AccelRange::G2).await?;

        // Конфигурация гироскопа (по умолчанию ±250°/s)
        self.set_gyro_range(i2c, GyroRange::Deg250).await?;

        // Включение всех осей
        self.write_register(i2c, regs::PWR_MGMT_2, 0x00).await?;

        // Небольшая задержка для стабилизации
        Timer::after(Duration::from_millis(20)).await;

        defmt::info!("MPU6050 инициализирован успешно");
        Ok(())
    }

    /// Установка диапазона измерения акселерометра
    pub async fn set_accel_range<I>(&mut self, i2c: &mut I2c<'_, I, Blocking>, range: AccelRange) -> Result<(), Mpu6050Error>
    where
        I: embassy_rp::i2c::Instance,
    {
        self.write_register(i2c, regs::ACCEL_CONFIG, range as u8).await?;
        self.accel_range = range;

        // Обновляем масштабный коэффициент
        self.accel_scale = match range {
            AccelRange::G2 => 16384.0,
            AccelRange::G4 => 8192.0,
            AccelRange::G8 => 4096.0,
            AccelRange::G16 => 2048.0,
        };

        Ok(())
    }

    /// Установка диапазона измерения гироскопа
    pub async fn set_gyro_range<I>(&mut self, i2c: &mut I2c<'_, I, Blocking>, range: GyroRange) -> Result<(), Mpu6050Error>
    where
        I: embassy_rp::i2c::Instance,
    {
        self.write_register(i2c, regs::GYRO_CONFIG, range as u8).await?;
        self.gyro_range = range;

        // Обновляем масштабный коэффициент
        self.gyro_scale = match range {
            GyroRange::Deg250 => 131.0,
            GyroRange::Deg500 => 65.5,
            GyroRange::Deg1000 => 32.8,
            GyroRange::Deg2000 => 16.4,
        };

        Ok(())
    }

    /// Чтение одного регистра
    async fn read_register<I>(&self, i2c: &mut I2c<'_, I, Blocking>, reg: u8) -> Result<u8, I2cError>
    where
        I: embassy_rp::i2c::Instance,
    {
        let mut buf = [0u8; 1];
        i2c.write_read(self.addr, &[reg], &mut buf)?;
        Ok(buf[0])
    }

    /// Запись в регистр
    async fn write_register<I>(&self, i2c: &mut I2c<'_, I, Blocking>, reg: u8, value: u8) -> Result<(), I2cError>
    where
        I: embassy_rp::i2c::Instance,
    {
        
        i2c.write(self.addr, &[reg, value])
    }

    /// Чтение нескольких регистров подряд
    async fn read_registers<I>(&self, i2c: &mut I2c<'_, I, Blocking>, start_reg: u8, buf: &mut [u8]) -> Result<(), I2cError>
    where
        I: embassy_rp::i2c::Instance,
    {
        i2c.write_read(self.addr, &[start_reg], buf)
    }

    /// Чтение сырых данных со всех датчиков
    pub async fn read_raw<I>(&self, i2c: &mut I2c<'_, I, Blocking>) -> Result<RawData, Mpu6050Error>
    where
        I: embassy_rp::i2c::Instance,
    {
        // Читаем все 14 байт данных одной операцией (с ACCEL_XOUT_H по GYRO_ZOUT_L)
        let mut buf = [0u8; 14];
        self.read_registers(i2c, regs::ACCEL_XOUT_H, &mut buf).await?;

        // Преобразуем байты в 16-битные значения со знаком (big-endian)
        Ok(RawData {
            accel_x_raw: i16::from_be_bytes([buf[0], buf[1]]),
            accel_y_raw: i16::from_be_bytes([buf[2], buf[3]]),
            accel_z_raw: i16::from_be_bytes([buf[4], buf[5]]),
            temp_raw: i16::from_be_bytes([buf[6], buf[7]]),
            gyro_x_raw: i16::from_be_bytes([buf[8], buf[9]]),
            gyro_y_raw: i16::from_be_bytes([buf[10], buf[11]]),
            gyro_z_raw: i16::from_be_bytes([buf[12], buf[13]]),
        })
    }

    /// Чтение и обработка всех данных
    pub async fn read_all<I>(&self, i2c: &mut I2c<'_, I, Blocking>) -> Result<ProcessedData, Mpu6050Error>
    where
        I: embassy_rp::i2c::Instance,
    {
        let raw = self.read_raw(i2c).await?;

        // Преобразование в физические единицы
        Ok(ProcessedData {
            // Акселерометр: преобразуем в м/с² (g = 9.81 м/с²)
            accel_x: (raw.accel_x_raw as f32 / self.accel_scale) * 9.81,
            accel_y: (raw.accel_y_raw as f32 / self.accel_scale) * 9.81,
            accel_z: (raw.accel_z_raw as f32 / self.accel_scale) * 9.81,

            // Гироскоп: преобразуем из °/с в рад/с
            gyro_x: (raw.gyro_x_raw as f32 / self.gyro_scale) * (core::f32::consts::PI / 180.0),
            gyro_y: (raw.gyro_y_raw as f32 / self.gyro_scale) * (core::f32::consts::PI / 180.0),
            gyro_z: (raw.gyro_z_raw as f32 / self.gyro_scale) * (core::f32::consts::PI / 180.0),

            // Температура: Temperature = 36.53 + (TEMP_OUT / 340)
            temperature: 36.53 + (raw.temp_raw as f32 / 340.0),
        })
    }

    /// Калибровка гироскопа (определение смещения нуля)
    pub async fn calibrate_gyro<I>(&self, i2c: &mut I2c<'_, I, Blocking>, samples: u16) -> Result<(f32, f32, f32), Mpu6050Error>
    where
        I: embassy_rp::i2c::Instance,
    {
        defmt::info!("Начало калибровки гироскопа, {} измерений", samples);

        let mut sum_x = 0i32;
        let mut sum_y = 0i32;
        let mut sum_z = 0i32;

        // Собираем данные
        for _ in 0..samples {
            let raw = self.read_raw(i2c).await?;
            sum_x += raw.gyro_x_raw as i32;
            sum_y += raw.gyro_y_raw as i32;
            sum_z += raw.gyro_z_raw as i32;

            // Небольшая задержка между измерениями
            Timer::after(Duration::from_millis(10)).await;
        }

        // Вычисляем средние значения смещения в рад/с
        let offset_x = (sum_x as f32 / samples as f32 / self.gyro_scale) * (core::f32::consts::PI / 180.0);
        let offset_y = (sum_y as f32 / samples as f32 / self.gyro_scale) * (core::f32::consts::PI / 180.0);
        let offset_z = (sum_z as f32 / samples as f32 / self.gyro_scale) * (core::f32::consts::PI / 180.0);

        defmt::info!("Калибровка завершена. Смещения: X={}, Y={}, Z={}", offset_x, offset_y, offset_z);

        Ok((offset_x, offset_y, offset_z))
    }

    /// Проверка готовности новых данных
    pub async fn data_ready<I>(&self, i2c: &mut I2c<'_, I, Blocking>) -> Result<bool, Mpu6050Error>
    where
        I: embassy_rp::i2c::Instance,
    {
        // Читаем регистр статуса прерываний
        let status = self.read_register(i2c, 0x3A).await?;
        Ok((status & 0x01) != 0)
    }
}