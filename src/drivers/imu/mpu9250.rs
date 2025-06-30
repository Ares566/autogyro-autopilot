//! Драйвер для MPU9250 (MPU6500 + AK8963)
use cortex_m::prelude::{_embedded_hal_blocking_i2c_Write, _embedded_hal_blocking_i2c_WriteRead};
use embassy_rp::i2c::{I2c, Error as I2cError, Blocking};
use embassy_time::{Duration, Timer, Instant};
use core::f32::consts::PI;
use num_traits::{Float, Zero};

/// Адреса устройств
const MPU9250_ADDR: u8 = 0x68;
const AK8963_ADDR: u8 = 0x0C;

/// Регистры MPU9250
#[allow(dead_code)]
mod regs {
    // Основные регистры (совместимы с MPU6050)
    pub const PWR_MGMT_1: u8 = 0x6B;
    pub const PWR_MGMT_2: u8 = 0x6C;
    pub const CONFIG: u8 = 0x1A;
    pub const GYRO_CONFIG: u8 = 0x1B;
    pub const ACCEL_CONFIG: u8 = 0x1C;
    pub const ACCEL_CONFIG_2: u8 = 0x1D;
    pub const SMPLRT_DIV: u8 = 0x19;
    pub const INT_ENABLE: u8 = 0x38;
    pub const INT_PIN_CFG: u8 = 0x37;
    pub const WHO_AM_I: u8 = 0x75;
    pub const USER_CTRL: u8 = 0x6A;

    // Регистры данных
    pub const ACCEL_XOUT_H: u8 = 0x3B;
    pub const TEMP_OUT_H: u8 = 0x41;
    pub const GYRO_XOUT_H: u8 = 0x43;

    // Регистры магнитометра AK8963
    pub mod ak8963 {
        pub const WIA: u8 = 0x00;      // Device ID (should be 0x48)
        pub const INFO: u8 = 0x01;     // Information
        pub const ST1: u8 = 0x02;      // Status 1
        pub const HXL: u8 = 0x03;      // X-axis data low
        pub const HXH: u8 = 0x04;      // X-axis data high
        pub const HYL: u8 = 0x05;      // Y-axis data low
        pub const HYH: u8 = 0x06;      // Y-axis data high
        pub const HZL: u8 = 0x07;      // Z-axis data low
        pub const HZH: u8 = 0x08;      // Z-axis data high
        pub const ST2: u8 = 0x09;      // Status 2
        pub const CNTL1: u8 = 0x0A;    // Control 1
        pub const CNTL2: u8 = 0x0B;    // Control 2
        pub const ASTC: u8 = 0x0C;     // Self-test
        pub const ASAX: u8 = 0x10;     // X-axis sensitivity adjustment
        pub const ASAY: u8 = 0x11;     // Y-axis sensitivity adjustment
        pub const ASAZ: u8 = 0x12;     // Z-axis sensitivity adjustment
    }
}

/// Диапазон измерения акселерометра
#[derive(Debug, Clone, Copy)]
pub enum AccelRange {
    G2 = 0x00,
    G4 = 0x08,
    G8 = 0x10,
    G16 = 0x18,
}

/// Диапазон измерения гироскопа
#[derive(Debug, Clone, Copy)]
pub enum GyroRange {
    Deg250 = 0x00,
    Deg500 = 0x08,
    Deg1000 = 0x10,
    Deg2000 = 0x18,
}

/// Режим работы магнитометра
#[derive(Debug, Clone, Copy)]
pub enum MagMode {
    PowerDown = 0x00,
    SingleMeasurement = 0x01,
    Continuous8Hz = 0x02,
    Continuous100Hz = 0x06,
    FuseRom = 0x0F,
}

/// Ошибки MPU9250
#[derive(Debug)]
pub enum Mpu9250Error {
    I2c(I2cError),
    InvalidDevice,
    InvalidMagnetometer,
    ConfigError,
    MagOverflow,
}

impl defmt::Format for Mpu9250Error {
    fn format(&self, fmt: defmt::Formatter) {
        match self {
            Mpu9250Error::I2c(_) => defmt::write!(fmt, "MPU9250: I2C error"),
            Mpu9250Error::InvalidDevice => defmt::write!(fmt, "MPU9250: Invalid device ID"),
            Mpu9250Error::InvalidMagnetometer => defmt::write!(fmt, "MPU9250: Invalid magnetometer ID"),
            Mpu9250Error::ConfigError => defmt::write!(fmt, "MPU9250: Configuration error"),
            Mpu9250Error::MagOverflow => defmt::write!(fmt, "MPU9250: Magnetometer overflow"),
        }
    }
}

impl From<I2cError> for Mpu9250Error {
    fn from(error: I2cError) -> Self {
        Mpu9250Error::I2c(error)
    }
}

/// Данные с всех датчиков MPU9250
#[derive(Debug, Clone, Copy)]
pub struct Mpu9250Data {
    /// Ускорение по X (м/с²)
    pub accel_x: f32,
    /// Ускорение по Y (м/с²)
    pub accel_y: f32,
    /// Ускорение по Z (м/с²)
    pub accel_z: f32,
    /// Угловая скорость по X (рад/с)
    pub gyro_x: f32,
    /// Угловая скорость по Y (рад/с)
    pub gyro_y: f32,
    /// Угловая скорость по Z (рад/с)
    pub gyro_z: f32,
    /// Магнитное поле по X (мкТл)
    pub mag_x: f32,
    /// Магнитное поле по Y (мкТл)
    pub mag_y: f32,
    /// Магнитное поле по Z (мкТл)
    pub mag_z: f32,
    /// Температура (°C)
    pub temperature: f32,
    /// Углы ориентации (рад)
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
}

/// Драйвер MPU9250
pub struct Mpu9250 {
    addr: u8,
    mag_addr: u8,
    accel_scale: f32,
    gyro_scale: f32,
    mag_sensitivity: (f32, f32, f32),
    mag_offset: (f32, f32, f32),
    gyro_offset: (f32, f32, f32),
}

impl Mpu9250 {
    /// Создание нового экземпляра драйвера
    pub async fn new<I>(i2c: &mut I2c<'_, I, Blocking>) -> Result<Self, Mpu9250Error>
    where
        I: embassy_rp::i2c::Instance,
    {
        let mut mpu = Self {
            addr: MPU9250_ADDR,
            mag_addr: AK8963_ADDR,
            accel_scale: 16384.0, // ±2g по умолчанию
            gyro_scale: 131.0,    // ±250°/s по умолчанию
            mag_sensitivity: (1.0, 1.0, 1.0),
            mag_offset: (0.0, 0.0, 0.0),
            gyro_offset: (0.0, 0.0, 0.0),
        };

        // Инициализация
        mpu.init(i2c).await?;

        Ok(mpu)
    }

    /// Полная инициализация MPU9250
    async fn init<I>(&mut self, i2c: &mut I2c<'_, I, Blocking>) -> Result<(), Mpu9250Error>
    where
        I: embassy_rp::i2c::Instance,
    {
        // Проверка ID устройства
        let who_am_i = self.read_register(i2c, regs::WHO_AM_I).await?;
        if who_am_i != 0x71 && who_am_i != 0x73 {
            defmt::error!("Неверный ID MPU9250: 0x{:02x}", who_am_i);
            return Err(Mpu9250Error::InvalidDevice);
        }

        // Сброс устройства
        self.write_register(i2c, regs::PWR_MGMT_1, 0x80).await?;
        Timer::after(Duration::from_millis(100)).await;

        // Выход из режима сна
        self.write_register(i2c, regs::PWR_MGMT_1, 0x01).await?;
        Timer::after(Duration::from_millis(50)).await;

        // Включение всех осей
        self.write_register(i2c, regs::PWR_MGMT_2, 0x00).await?;

        // Конфигурация: DLPF = 3 (41Hz gyro, 42Hz accel)
        self.write_register(i2c, regs::CONFIG, 0x03).await?;

        // Частота выборки = 100Hz
        self.write_register(i2c, regs::SMPLRT_DIV, 9).await?;

        // Конфигурация акселерометра (±2g)
        self.set_accel_range(i2c, AccelRange::G2).await?;
        self.write_register(i2c, regs::ACCEL_CONFIG_2, 0x03).await?; // 41Hz LPF

        // Конфигурация гироскопа (±250°/s)
        self.set_gyro_range(i2c, GyroRange::Deg250).await?;

        // Включение bypass для доступа к магнитометру
        self.write_register(i2c, regs::INT_PIN_CFG, 0x02).await?;
        Timer::after(Duration::from_millis(10)).await;

        // Инициализация магнитометра
        self.init_magnetometer(i2c).await?;

        defmt::info!("MPU9250 инициализирован успешно");
        Ok(())
    }

    /// Инициализация магнитометра AK8963
    async fn init_magnetometer<I>(&mut self, i2c: &mut I2c<'_, I, Blocking>) -> Result<(), Mpu9250Error>
    where
        I: embassy_rp::i2c::Instance,
    {
        // Проверка ID магнитометра
        let mag_id = self.read_mag_register(i2c, regs::ak8963::WIA).await?;
        if mag_id != 0x48 {
            defmt::error!("Неверный ID AK8963: 0x{:02x}", mag_id);
            return Err(Mpu9250Error::InvalidMagnetometer);
        }

        // Сброс магнитометра
        self.write_mag_register(i2c, regs::ak8963::CNTL2, 0x01).await?;
        Timer::after(Duration::from_millis(10)).await;

        // Чтение калибровочных данных
        self.write_mag_register(i2c, regs::ak8963::CNTL1, MagMode::FuseRom as u8).await?;
        Timer::after(Duration::from_millis(10)).await;

        let asa_x = self.read_mag_register(i2c, regs::ak8963::ASAX).await?;
        let asa_y = self.read_mag_register(i2c, regs::ak8963::ASAY).await?;
        let asa_z = self.read_mag_register(i2c, regs::ak8963::ASAZ).await?;

        // Расчет коэффициентов чувствительности
        self.mag_sensitivity = (
            ((asa_x as f32 - 128.0) / 256.0 + 1.0),
            ((asa_y as f32 - 128.0) / 256.0 + 1.0),
            ((asa_z as f32 - 128.0) / 256.0 + 1.0),
        );

        // Установка режима непрерывного измерения 100Hz, 16-bit
        self.write_mag_register(i2c, regs::ak8963::CNTL1, 0x16).await?;
        Timer::after(Duration::from_millis(10)).await;

        defmt::info!("AK8963 инициализирован, чувствительность: X={}, Y={}, Z={}", 
            self.mag_sensitivity.0, self.mag_sensitivity.1, self.mag_sensitivity.2);

        Ok(())
    }

    /// Чтение всех данных
    pub async fn read_all<I>(&self, i2c: &mut I2c<'_, I, Blocking>) -> Result<Mpu9250Data, Mpu9250Error>
    where
        I: embassy_rp::i2c::Instance,
    {
        // Чтение акселерометра и гироскопа (14 байт)
        let mut buf = [0u8; 14];
        self.read_registers(i2c, regs::ACCEL_XOUT_H, &mut buf).await?;

        // Преобразование данных
        let accel_x_raw = i16::from_be_bytes([buf[0], buf[1]]);
        let accel_y_raw = i16::from_be_bytes([buf[2], buf[3]]);
        let accel_z_raw = i16::from_be_bytes([buf[4], buf[5]]);
        let temp_raw = i16::from_be_bytes([buf[6], buf[7]]);
        let gyro_x_raw = i16::from_be_bytes([buf[8], buf[9]]);
        let gyro_y_raw = i16::from_be_bytes([buf[10], buf[11]]);
        let gyro_z_raw = i16::from_be_bytes([buf[12], buf[13]]);

        // Чтение магнитометра
        let (mag_x_raw, mag_y_raw, mag_z_raw) = self.read_magnetometer(i2c).await?;

        // Преобразование в физические единицы
        let accel_x = (accel_x_raw as f32 / self.accel_scale) * 9.81;
        let accel_y = (accel_y_raw as f32 / self.accel_scale) * 9.81;
        let accel_z = (accel_z_raw as f32 / self.accel_scale) * 9.81;

        let gyro_x = ((gyro_x_raw as f32 / self.gyro_scale) * PI / 180.0) - self.gyro_offset.0;
        let gyro_y = ((gyro_y_raw as f32 / self.gyro_scale) * PI / 180.0) - self.gyro_offset.1;
        let gyro_z = ((gyro_z_raw as f32 / self.gyro_scale) * PI / 180.0) - self.gyro_offset.2;

        // Магнитометр с учетом чувствительности и смещения
        let mag_x = mag_x_raw * self.mag_sensitivity.0 - self.mag_offset.0;
        let mag_y = mag_y_raw * self.mag_sensitivity.1 - self.mag_offset.1;
        let mag_z = mag_z_raw * self.mag_sensitivity.2 - self.mag_offset.2;

        let temperature = (temp_raw as f32 / 333.87) + 21.0;

        // Расчет углов ориентации
        let roll = accel_y.atan2(accel_z);
        let pitch = (-accel_x).atan2((accel_y * accel_y + accel_z * accel_z).sqrt());

        // Расчет курса по магнитометру (упрощенный, без наклона)
        let yaw = (-mag_y).atan2(mag_x);

        Ok(Mpu9250Data {
            accel_x,
            accel_y,
            accel_z,
            gyro_x,
            gyro_y,
            gyro_z,
            mag_x,
            mag_y,
            mag_z,
            temperature,
            roll,
            pitch,
            yaw,
        })
    }

    /// Чтение данных магнитометра
    async fn read_magnetometer<I>(&self, i2c: &mut I2c<'_, I, Blocking>) -> Result<(f32, f32, f32), Mpu9250Error>
    where
        I: embassy_rp::i2c::Instance,
    {
        // Проверка готовности данных
        let st1 = self.read_mag_register(i2c, regs::ak8963::ST1).await?;
        if (st1 & 0x01) == 0 {
            // Данные не готовы, возвращаем последние
            return Ok((0.0, 0.0, 0.0));
        }

        // Чтение 7 байт (6 байт данных + ST2)
        let mut buf = [0u8; 7];
        i2c.write_read(self.mag_addr, &[regs::ak8963::HXL], &mut buf)?;

        // Проверка переполнения
        if (buf[6] & 0x08) != 0 {
            return Err(Mpu9250Error::MagOverflow);
        }

        // Преобразование данных
        let mag_x = i16::from_le_bytes([buf[0], buf[1]]) as f32 * 0.15; // 0.15 мкТл/LSB
        let mag_y = i16::from_le_bytes([buf[2], buf[3]]) as f32 * 0.15;
        let mag_z = i16::from_le_bytes([buf[4], buf[5]]) as f32 * 0.15;

        Ok((mag_x, mag_y, mag_z))
    }

    /// Калибровка гироскопа
    pub async fn calibrate_gyro<I>(&mut self, i2c: &mut I2c<'_, I, Blocking>, samples: u16) -> Result<(f32, f32, f32), Mpu9250Error>
    where
        I: embassy_rp::i2c::Instance,
    {
        defmt::info!("Калибровка гироскопа, {} измерений", samples);

        let mut sum_x = 0f32;
        let mut sum_y = 0f32;
        let mut sum_z = 0f32;

        for _ in 0..samples {
            let data = self.read_all(i2c).await?;
            sum_x += data.gyro_x;
            sum_y += data.gyro_y;
            sum_z += data.gyro_z;
            Timer::after(Duration::from_millis(10)).await;
        }

        self.gyro_offset = (
            sum_x / samples as f32,
            sum_y / samples as f32,
            sum_z / samples as f32,
        );

        defmt::info!("Смещения гироскопа: X={}, Y={}, Z={}", 
            self.gyro_offset.0, self.gyro_offset.1, self.gyro_offset.2);

        Ok(self.gyro_offset)
    }

    /// Калибровка магнитометра (hard iron)
    pub async fn calibrate_magnetometer<I>(&mut self, i2c: &mut I2c<'_, I, Blocking>, duration_s: u32) -> Result<(), Mpu9250Error>
    where
        I: embassy_rp::i2c::Instance,
    {
        defmt::info!("Калибровка магнитометра, вращайте устройство {} секунд", duration_s);

        let mut min_x = f32::MAX;
        let mut max_x = f32::MIN;
        let mut min_y = f32::MAX;
        let mut max_y = f32::MIN;
        let mut min_z = f32::MAX;
        let mut max_z = f32::MIN;

        let start = Instant::now();

        while start.elapsed().as_secs() < duration_s as u64 {
            let (mag_x, mag_y, mag_z) = self.read_magnetometer(i2c).await?;

            min_x = min_x.min(mag_x);
            max_x = max_x.max(mag_x);
            min_y = min_y.min(mag_y);
            max_y = max_y.max(mag_y);
            min_z = min_z.min(mag_z);
            max_z = max_z.max(mag_z);

            Timer::after(Duration::from_millis(50)).await;
        }

        // Расчет смещений (hard iron)
        self.mag_offset = (
            (max_x + min_x) / 2.0,
            (max_y + min_y) / 2.0,
            (max_z + min_z) / 2.0,
        );

        defmt::info!("Калибровка завершена. Смещения: X={}, Y={}, Z={}", 
            self.mag_offset.0, self.mag_offset.1, self.mag_offset.2);

        Ok(())
    }

    /// Установка диапазона акселерометра
    pub async fn set_accel_range<I>(&mut self, i2c: &mut I2c<'_, I, Blocking>, range: AccelRange) -> Result<(), Mpu9250Error>
    where
        I: embassy_rp::i2c::Instance,
    {
        self.write_register(i2c, regs::ACCEL_CONFIG, range as u8).await?;

        self.accel_scale = match range {
            AccelRange::G2 => 16384.0,
            AccelRange::G4 => 8192.0,
            AccelRange::G8 => 4096.0,
            AccelRange::G16 => 2048.0,
        };

        Ok(())
    }

    /// Установка диапазона гироскопа
    pub async fn set_gyro_range<I>(&mut self, i2c: &mut I2c<'_, I, Blocking>, range: GyroRange) -> Result<(), Mpu9250Error>
    where
        I: embassy_rp::i2c::Instance,
    {
        self.write_register(i2c, regs::GYRO_CONFIG, range as u8).await?;

        self.gyro_scale = match range {
            GyroRange::Deg250 => 131.0,
            GyroRange::Deg500 => 65.5,
            GyroRange::Deg1000 => 32.8,
            GyroRange::Deg2000 => 16.4,
        };

        Ok(())
    }

    // Вспомогательные методы I2C
    async fn read_register<I>(&self, i2c: &mut I2c<'_, I, Blocking>, reg: u8) -> Result<u8, I2cError>
    where
        I: embassy_rp::i2c::Instance,
    {
        let mut buf = [0u8; 1];
        i2c.write_read(self.addr, &[reg], &mut buf)?;
        Ok(buf[0])
    }

    async fn write_register<I>(&self, i2c: &mut I2c<'_, I, Blocking>, reg: u8, value: u8) -> Result<(), I2cError>
    where
        I: embassy_rp::i2c::Instance,
    {
        i2c.write(self.addr, &[reg, value])
    }

    async fn read_registers<I>(&self, i2c: &mut I2c<'_, I, Blocking>, start_reg: u8, buf: &mut [u8]) -> Result<(), I2cError>
    where
        I: embassy_rp::i2c::Instance,
    {
        i2c.write_read(self.addr, &[start_reg], buf)
    }

    async fn read_mag_register<I>(&self, i2c: &mut I2c<'_, I, Blocking>, reg: u8) -> Result<u8, I2cError>
    where
        I: embassy_rp::i2c::Instance,
    {
        let mut buf = [0u8; 1];
        i2c.write_read(self.mag_addr, &[reg], &mut buf)?;
        Ok(buf[0])
    }

    async fn write_mag_register<I>(&self, i2c: &mut I2c<'_, I, Blocking>, reg: u8, value: u8) -> Result<(), I2cError>
    where
        I: embassy_rp::i2c::Instance,
    {
        i2c.write(self.mag_addr, &[reg, value])
    }
}
