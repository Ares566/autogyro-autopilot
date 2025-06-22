use embassy_time::{Duration, Timer, Instant};
use embassy_rp::i2c::{I2c, Config, Blocking};
use embassy_rp::peripherals::I2C0;

use crate::data::{CHANNELS, SYSTEM_STATE, ImuData};
use crate::drivers::imu::mpu6050::Mpu6050;

use num_traits::{Float, Zero};

/// Частота опроса IMU (Гц)
const IMU_UPDATE_RATE_HZ: u32 = 100;

#[embassy_executor::task]
pub async fn task(mut i2c: I2c<'static, I2C0, Blocking>) {
    // Инициализация IMU
    let mut imu = match Mpu6050::new(&mut i2c).await {
        Ok(imu) => {
            defmt::info!("IMU инициализирован успешно");
            imu
        }
        Err(e) => {
            defmt::error!("Ошибка инициализации IMU: {:?}", e);
            // TODO Переход в аварийный режим
            //*SYSTEM_STATE.flight_mode.lock().await = crate::data::FlightMode::Emergency;
            return;
        }
    };

    // Получаем отправитель канала
    let imu_sender = CHANNELS.imu_channel.sender();

    let mut ticker = embassy_time::Ticker::every(Duration::from_hz(IMU_UPDATE_RATE_HZ as u64));

    loop {
        ticker.next().await;

        // Считываем данные с IMU
        match imu.read_all(&mut i2c).await {
            Ok(raw_data) => {
                // Преобразуем сырые данные в углы и угловые скорости
                let imu_data = ImuData {
                    roll: raw_data.accel_x.atan2(raw_data.accel_z),
                    pitch: (-raw_data.accel_y).atan2(
                        (raw_data.accel_x * raw_data.accel_x + raw_data.accel_z * raw_data.accel_z).sqrt()
                    ),
                    yaw: 0.0, // Без магнитометра используем интегрирование гироскопа
                    roll_rate: raw_data.gyro_x,
                    pitch_rate: raw_data.gyro_y,
                    yaw_rate: raw_data.gyro_z,
                    timestamp_us: Instant::now().as_micros(),
                };

                // Отправляем в канал (неблокирующий вариант)
                if let Err(_) = imu_sender.try_send(imu_data) {
                    defmt::warn!("Буфер IMU канала переполнен");
                }

                // Обновляем последнее значение в общем состоянии
                *SYSTEM_STATE.last_imu.lock().await = Some(imu_data);

            }
            Err(e) => {
                defmt::error!("Ошибка чтения IMU: {:?}", e);
                // Можно добавить счетчик ошибок и переход в аварийный режим
            }
        }
    }
}