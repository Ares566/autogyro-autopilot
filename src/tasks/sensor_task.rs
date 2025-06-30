use embassy_time::{Duration, Timer, Instant};
use embassy_rp::i2c::{I2c, Config, Blocking};
use embassy_rp::peripherals::I2C0;

use crate::data::{CHANNELS, SYSTEM_STATE, ImuData};
use crate::drivers::imu::mpu6050::Mpu6050;

use num_traits::{Float, Zero};
use crate::drivers::baro::Bmp280;
use crate::drivers::imu::{Mpu6500, Mpu9250};
use crate::drivers::mag::Hmc5983;
use crate::sensors::fusion::ComplementaryFilter;

/// Частота опроса IMU (Гц)
const IMU_UPDATE_RATE_HZ: u32 = 50;

#[embassy_executor::task]
pub async fn task(mut i2c: I2c<'static, I2C0, Blocking>) {
    // Инициализация IMU
    // let mut imu = match Mpu6050::new(&mut i2c).await {
    //     Ok(imu) => {
    //         defmt::info!("IMU инициализирован успешно");
    //         imu
    //     }
    //     Err(e) => {
    //         defmt::error!("Ошибка инициализации IMU: {:?}", e);
    //         // TODO Переход в аварийный режим
    //         //*SYSTEM_STATE.flight_mode.lock().await = crate::data::FlightMode::Emergency;
    //         return;
    //     }
    // };
    // 
    

    // Инициализация BMP280 (барометр)
    let mut baro = match Bmp280::new(&mut i2c, 0x76).await {
        Ok(baro) => {
            defmt::info!("BMP280 инициализирован успешно");
            baro
        }
        Err(e) => {
            defmt::error!("Ошибка инициализации BMP280: {}", e);
            return;
        }
    };
    // Инициализация MPU6500 (без магнитометра)
    let mut imu = match Mpu6500::new(&mut i2c).await {
        Ok(imu) => {
            defmt::info!("MPU6500 инициализирован успешно");
            imu
        }
        Err(e) => {
            defmt::error!("Ошибка инициализации MPU6500: {}", e);
            return;
        }
    };

    // Инициализация HMC5983
    let mut mag = match Hmc5983::new(&mut i2c).await {
        Ok(mut mag) => {
            // Установка магнитного склонения для вашего региона
            // Например, для Москвы около +11°
            mag.set_declination(11.0);
            defmt::info!("HMC5983 инициализирован успешно");
            mag
        }
        Err(e) => {
            defmt::error!("Ошибка инициализации HMC5983: {}", e);
            return;
        }
    };
    
    // Калибровка IMU
    defmt::info!("Калибровка IMU, не двигайте устройство...");
    let gyro_offsets = imu.calibrate_gyro(&mut i2c, 100).await.unwrap();
    defmt::info!("Калибровка завершена");

    // Опционально: самотестирование
    // if let Ok(test_passed) = imu.self_test(&mut i2c).await {
    //     if !test_passed {
    //         defmt::warn!("IMU не прошел самотестирование!");
    //     }
    // }

    // Калибровка магнитометра (опционально)
    // defmt::info!("Калибровка магнитометра...");
    // mag.calibrate(&mut i2c, 30).await.unwrap();
    
    // Получаем отправитель канала
    let imu_sender = CHANNELS.imu_channel.sender();

    // Создаем фильтр для объединения данных
    let mut filter = ComplementaryFilter::new();

    let mut ticker = embassy_time::Ticker::every(Duration::from_hz(100));

    loop {
        ticker.next().await;

        // Читаем IMU
        if let Ok(imu_data) = imu.read_all(&mut i2c).await {
            // Читаем магнитометр (реже - 20Hz)
            static mut MAG_COUNTER: u8 = 0;
            let mag_heading = unsafe {
                MAG_COUNTER += 1;
                if MAG_COUNTER >= 5 {
                    MAG_COUNTER = 0;
                    if let Ok(mag_data) = mag.read(&mut i2c).await {
                        Some(mag_data.heading)
                    } else {
                        None
                    }
                } else {
                    None
                }
            };

            // Объединяем данные с помощью фильтра
            let fused_data = filter.update(
                &imu_data,
                mag_heading,
                0.01, // dt = 10ms при 100Hz
            );

            // Отправляем объединенные данные
            let imu_msg = ImuData {
                roll: fused_data.roll,
                pitch: fused_data.pitch,
                yaw: fused_data.yaw,
                roll_rate: imu_data.gyro_x,
                pitch_rate: imu_data.gyro_y,
                yaw_rate: imu_data.gyro_z,
                timestamp_us: Instant::now().as_micros(),
            };

            let _ = CHANNELS.imu_channel.try_send(imu_msg);
            *SYSTEM_STATE.last_imu.lock().await = Some(imu_msg);
        }
    }
    
    // let mut ticker = embassy_time::Ticker::every(Duration::from_hz(IMU_UPDATE_RATE_HZ as u64));
    // 
    // loop {
    //     ticker.next().await;
    // 
    //     // Считываем данные с IMU
    //     match imu.read_all(&mut i2c).await {
    //         Ok(raw_data) => {
    //             // Преобразуем сырые данные в углы и угловые скорости
    //             let imu_data = ImuData {
    //                 roll: raw_data.accel_x.atan2(raw_data.accel_z),
    //                 pitch: (-raw_data.accel_y).atan2(
    //                     (raw_data.accel_x * raw_data.accel_x + raw_data.accel_z * raw_data.accel_z).sqrt()
    //                 ),
    //                 yaw: 0.0, // Без магнитометра используем интегрирование гироскопа
    //                 roll_rate: raw_data.gyro_x,
    //                 pitch_rate: raw_data.gyro_y,
    //                 yaw_rate: raw_data.gyro_z,
    //                 timestamp_us: Instant::now().as_micros(),
    //             };
    // 
    //             // Отправляем в канал (неблокирующий вариант)
    //             if let Err(_) = imu_sender.try_send(imu_data) {
    //                 defmt::warn!("Буфер IMU канала переполнен");
    //             }
    // 
    //             // Обновляем последнее значение в общем состоянии
    //             *SYSTEM_STATE.last_imu.lock().await = Some(imu_data);
    // 
    //             // Логирование для отладки
    //             #[cfg(feature = "debug-sensors")]
    //             defmt::debug!(
    //                 "Sensors: P={} R={} Y={}",
    //                 imu_data.pitch,
    //                 imu_data.roll,
    //                 imu_data.yaw,
    //             );
    //         }
    //         Err(e) => {
    //             defmt::error!("Ошибка чтения IMU: {:?}", e);
    //             // Можно добавить счетчик ошибок и переход в аварийный режим
    //         }
    //     }
    // }
}