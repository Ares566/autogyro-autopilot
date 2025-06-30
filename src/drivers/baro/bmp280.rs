//! Драйвер для барометра BMP280
use cortex_m::prelude::{_embedded_hal_blocking_i2c_Write, _embedded_hal_blocking_i2c_WriteRead};
use embassy_rp::i2c::{I2c, Error as I2cError, Blocking};
use embassy_time::{Duration, Timer};

/// Регистры BMP280
#[allow(dead_code)]
mod regs {
    pub const ID: u8 = 0xD0;           // Chip ID
    pub const RESET: u8 = 0xE0;        // Reset register
    pub const STATUS: u8 = 0xF3;       // Status register
    pub const CTRL_MEAS: u8 = 0xF4;    // Control measurement
    pub const CONFIG: u8 = 0xF5;       // Configuration

    // Регистры данных
    pub const PRESS_MSB: u8 = 0xF7;    // Pressure MSB
    pub const PRESS_LSB: u8 = 0xF8;    // Pressure LSB
    pub const PRESS_XLSB: u8 = 0xF9;   // Pressure XLSB
    pub const TEMP_MSB: u8 = 0xFA;     // Temperature MSB
    pub const TEMP_LSB: u8 = 0xFB;     // Temperature LSB
    pub const TEMP_XLSB: u8 = 0xFC;    // Temperature XLSB

    // Калибровочные регистры
    pub const CALIB_00: u8 = 0x88;     // Начало калибровочных данных
}

/// Режим работы
#[derive(Debug, Clone, Copy)]
pub enum Mode {
    Sleep = 0x00,
    Forced = 0x01,
    Normal = 0x03,
}

/// Передискретизация (oversampling)
#[derive(Debug, Clone, Copy)]
pub enum Oversampling {
    Skip = 0x00,
    X1 = 0x01,
    X2 = 0x02,
    X4 = 0x03,
    X8 = 0x04,
    X16 = 0x05,
}

/// Фильтр IIR
#[derive(Debug, Clone, Copy)]
pub enum IirFilter {
    Off = 0x00,
    X2 = 0x01,
    X4 = 0x02,
    X8 = 0x03,
    X16 = 0x04,
}

/// Время ожидания в нормальном режиме
#[derive(Debug, Clone, Copy)]
pub enum StandbyTime {
    Ms0_5 = 0x00,
    Ms62_5 = 0x01,
    Ms125 = 0x02,
    Ms250 = 0x03,
    Ms500 = 0x04,
    Ms1000 = 0x05,
    Ms2000 = 0x06,
    Ms4000 = 0x07,
}

/// Ошибки BMP280
#[derive(Debug)]
pub enum Bmp280Error {
    I2c(I2cError),
    InvalidDevice,
    CalibrationError,
}

impl defmt::Format for Bmp280Error {
    fn format(&self, fmt: defmt::Formatter) {
        match self {
            Bmp280Error::I2c(_) => defmt::write!(fmt, "BMP280: I2C error"),
            Bmp280Error::InvalidDevice => defmt::write!(fmt, "BMP280: Invalid device ID"),
            Bmp280Error::CalibrationError => defmt::write!(fmt, "BMP280: Calibration error"),
        }
    }
}

impl From<I2cError> for Bmp280Error {
    fn from(error: I2cError) -> Self {
        Bmp280Error::I2c(error)
    }
}

/// Калибровочные данные
#[derive(Debug, Clone, Copy)]
struct CalibrationData {
    dig_t1: u16,
    dig_t2: i16,
    dig_t3: i16,
    dig_p1: u16,
    dig_p2: i16,
    dig_p3: i16,
    dig_p4: i16,
    dig_p5: i16,
    dig_p6: i16,
    dig_p7: i16,
    dig_p8: i16,
    dig_p9: i16,
}

/// Драйвер BMP280
pub struct Bmp280 {
    addr: u8,
    calib: CalibrationData,
    sea_level_pa: f32,
}

impl Bmp280 {
    /// Создание нового экземпляра драйвера
    pub async fn new<I>(i2c: &mut I2c<'_, I, Blocking>, addr: u8) -> Result<Self, Bmp280Error>
    where
        I: embassy_rp::i2c::Instance,
    {
        let mut bmp = Self {
            addr,
            calib: CalibrationData {
                dig_t1: 0,
                dig_t2: 0,
                dig_t3: 0,
                dig_p1: 0,
                dig_p2: 0,
                dig_p3: 0,
                dig_p4: 0,
                dig_p5: 0,
                dig_p6: 0,
                dig_p7: 0,
                dig_p8: 0,
                dig_p9: 0,
            },
            sea_level_pa: 101325.0, // Стандартное давление на уровне моря
        };

        // Инициализация
        bmp.init(i2c).await?;

        Ok(bmp)
    }

    /// Инициализация BMP280
    async fn init<I>(&mut self, i2c: &mut I2c<'_, I, Blocking>) -> Result<(), Bmp280Error>
    where
        I: embassy_rp::i2c::Instance,
    {
        // Проверка ID
        let id = self.read_register(i2c, regs::ID).await?;
        if id != 0x58 {
            defmt::error!("Неверный ID BMP280: 0x{:02x}", id);
            return Err(Bmp280Error::InvalidDevice);
        }

        // Сброс
        self.write_register(i2c, regs::RESET, 0xB6).await?;
        Timer::after(Duration::from_millis(10)).await;

        // Чтение калибровочных данных
        self.read_calibration(i2c).await?;

        // Конфигурация
        // Нормальный режим, oversampling x16 для температуры и давления
        let ctrl_meas = (Oversampling::X16 as u8) << 5 | // osrs_t
            (Oversampling::X16 as u8) << 2 | // osrs_p
            Mode::Normal as u8;               // mode
        self.write_register(i2c, regs::CTRL_MEAS, ctrl_meas).await?;

        // Конфигурация: standby 125ms, filter x4
        let config = (StandbyTime::Ms125 as u8) << 5 |  // t_sb
            (IirFilter::X4 as u8) << 2;         // filter
        self.write_register(i2c, regs::CONFIG, config).await?;

        Timer::after(Duration::from_millis(50)).await;

        defmt::info!("BMP280 инициализирован успешно");
        Ok(())
    }

    /// Чтение калибровочных данных
    async fn read_calibration<I>(&mut self, i2c: &mut I2c<'_, I, Blocking>) -> Result<(), Bmp280Error>
    where
        I: embassy_rp::i2c::Instance,
    {
        let mut buf = [0u8; 24];
        self.read_registers(i2c, regs::CALIB_00, &mut buf).await?;

        self.calib.dig_t1 = u16::from_le_bytes([buf[0], buf[1]]);
        self.calib.dig_t2 = i16::from_le_bytes([buf[2], buf[3]]);
        self.calib.dig_t3 = i16::from_le_bytes([buf[4], buf[5]]);
        self.calib.dig_p1 = u16::from_le_bytes([buf[6], buf[7]]);
        self.calib.dig_p2 = i16::from_le_bytes([buf[8], buf[9]]);
        self.calib.dig_p3 = i16::from_le_bytes([buf[10], buf[11]]);
        self.calib.dig_p4 = i16::from_le_bytes([buf[12], buf[13]]);
        self.calib.dig_p5 = i16::from_le_bytes([buf[14], buf[15]]);
        self.calib.dig_p6 = i16::from_le_bytes([buf[16], buf[17]]);
        self.calib.dig_p7 = i16::from_le_bytes([buf[18], buf[19]]);
        self.calib.dig_p8 = i16::from_le_bytes([buf[20], buf[21]]);
        self.calib.dig_p9 = i16::from_le_bytes([buf[22], buf[23]]);

        Ok(())
    }

    /// Чтение температуры и давления
    pub async fn read<I>(&self, i2c: &mut I2c<'_, I, Blocking>) -> Result<(f32, f32), Bmp280Error>
    where
        I: embassy_rp::i2c::Instance,
    {
        // Проверка готовности данных
        let status = self.read_register(i2c, regs::STATUS).await?;
        if (status & 0x08) != 0 {
            // Измерение в процессе, ждем
            Timer::after(Duration::from_millis(10)).await;
        }

        // Чтение сырых данных
        let mut buf = [0u8; 6];
        self.read_registers(i2c, regs::PRESS_MSB, &mut buf).await?;

        let press_raw = ((buf[0] as u32) << 12) | ((buf[1] as u32) << 4) | ((buf[2] as u32) >> 4);
        let temp_raw = ((buf[3] as u32) << 12) | ((buf[4] as u32) << 4) | ((buf[5] as u32) >> 4);

        // Компенсация температуры
        let (temperature, t_fine) = self.compensate_temperature(temp_raw as i32);

        // Компенсация давления
        let pressure = self.compensate_pressure(press_raw as i32, t_fine);

        Ok((temperature, pressure))
    }

    /// Чтение высоты
    pub async fn read_altitude<I>(&self, i2c: &mut I2c<'_, I, Blocking>) -> Result<(f32, f32, f32), Bmp280Error>
    where
        I: embassy_rp::i2c::Instance,
    {
        let (temperature, pressure) = self.read(i2c).await?;

        // Расчет высоты по барометрической формуле
        let altitude = 44330.0 * (1.0 - libm::powf(pressure / self.sea_level_pa, 0.1903));

        Ok((altitude, pressure, temperature))
    }

    /// Установка давления на уровне моря для расчета высоты
    pub fn set_sea_level_pressure(&mut self, pressure_pa: f32) {
        self.sea_level_pa = pressure_pa;
    }

    /// Компенсация температуры (из datasheet)
    fn compensate_temperature(&self, adc_t: i32) -> (f32, i32) {
        let var1 = ((adc_t >> 3) - ((self.calib.dig_t1 as i32) << 1)) * (self.calib.dig_t2 as i32) >> 11;
        let var2 = (((((adc_t >> 4) - (self.calib.dig_t1 as i32)) *
            ((adc_t >> 4) - (self.calib.dig_t1 as i32))) >> 12) *
            (self.calib.dig_t3 as i32)) >> 14;
        let t_fine = var1 + var2;
        let temperature = ((t_fine * 5 + 128) >> 8) as f32 / 100.0;

        (temperature, t_fine)
    }

    /// Компенсация давления (из datasheet)
    fn compensate_pressure(&self, adc_p: i32, t_fine: i32) -> f32 {
        let mut var1 = ((t_fine as i64) >> 1) - 64000;
        let mut var2 = var1 * var1 * (self.calib.dig_p6 as i64) >> 15;
        var2 = var2 + ((var1 * (self.calib.dig_p5 as i64)) << 1);
        var2 = (var2 >> 2) + ((self.calib.dig_p4 as i64) << 16);
        var1 = (((self.calib.dig_p3 as i64) * ((var1 * var1) >> 13)) >> 3) +
            (((self.calib.dig_p2 as i64) * var1) >> 1);
        var1 = var1 >> 18;
        var1 = ((32768 + var1) * (self.calib.dig_p1 as i64)) >> 15;

        if var1 == 0 {
            return 0.0; // Избегаем деления на ноль
        }

        let mut p = ((1048576 - adc_p) as i64 - (var2 >> 12)) * 3125;
        if p < 0x80000000 {
            p = (p << 1) / var1;
        } else {
            p = (p / var1) * 2;
        }

        var1 = ((self.calib.dig_p9 as i64) * ((p >> 3) * (p >> 3)) >> 13) >> 12;
        var2 = ((p >> 2) * (self.calib.dig_p8 as i64)) >> 13;
        p = p + ((var1 + var2 + (self.calib.dig_p7 as i64)) >> 4);

        p as f32
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
}
