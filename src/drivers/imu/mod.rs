pub(crate) mod mpu6050;


pub use mpu6050::{Mpu6050, Mpu6050Error, ProcessedData as ImuProcessedData};

/// Трейт для всех IMU датчиков
pub trait ImuSensor {
    type Error;

    /// Чтение обработанных данных
    async fn read_processed(&mut self) -> Result<ImuProcessedData, Self::Error>;

    /// Калибровка датчика
    async fn calibrate(&mut self) -> Result<(), Self::Error>;
}