pub(crate) mod mpu6050;
mod mpu9250;
pub(crate) mod mpu6500;
// Экспортируем MPU6500 как основной для GY-91
pub use mpu6500::{Mpu6500, Mpu6500Error, Mpu6500Data};
pub use mpu6050::{Mpu6050, Mpu6050Error, ProcessedData as ImuProcessedData};
pub use mpu9250::{Mpu9250, Mpu9250Error, Mpu9250Data};

/// Трейт для всех IMU датчиков
pub trait ImuSensor {
    type Error;

    /// Чтение обработанных данных
    async fn read_processed(&mut self) -> Result<ImuProcessedData, Self::Error>;

    /// Калибровка датчика
    async fn calibrate(&mut self) -> Result<(), Self::Error>;
}