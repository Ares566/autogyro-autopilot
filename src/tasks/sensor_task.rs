use embassy_rp::i2c::{Blocking, Config, I2c};
use embassy_rp::peripherals::I2C0;
use embassy_time::{Duration, Instant, Timer};

use crate::data::{AltitudeData, ImuData, CHANNELS, SYSTEM_STATE};
use crate::drivers::imu::mpu6050::Mpu6050;

use crate::drivers::baro::Bmp280;
use crate::drivers::imu::{Mpu6500, Mpu9250};
use crate::drivers::mag::Hmc5983;
use crate::sensors::altitude::AltitudeProcessor;
use crate::sensors::fusion::ComplementaryFilter;
use num_traits::{Float, Zero};

/// Частота опроса IMU (Гц)
use crate::config::hardware::system::{BARO_SAMPLE_RATE_HZ, IMU_SAMPLE_RATE_HZ};

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
            mag.set_declination(8.0);
            defmt::info!("HMC5983 инициализирован успешно");
            mag
        }
        Err(e) => {
            defmt::error!("Ошибка инициализации HMC5983: {}", e);
            return;
        }
    };

    // === Калибровка датчиков ===

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
    defmt::info!("Калибровка магнитометра...");
    mag.calibrate(&mut i2c, 10).await.unwrap();

    // Калибровка барометра - устанавливаем текущее давление как уровень земли
    defmt::info!("Калибровка барометра...");
    if let Ok((_, pressure)) = baro.read(&mut i2c).await {
        baro.set_sea_level_pressure(pressure);
        defmt::info!("Барометр откалиброван, давление на земле: {} Па", pressure);
    }

    // Получаем отправитель канала
    let imu_sender = CHANNELS.imu_channel.sender();

    // === Инициализация обработчиков данных ===

    let mut fusion_filter = ComplementaryFilter::new();
    let mut altitude_processor = AltitudeProcessor::new();

    // Калибровка высоты
    if let Ok((_, pressure)) = baro.read(&mut i2c).await {
        altitude_processor.calibrate_ground_level(pressure);
    }

    // === Основной цикл опроса ===
    let mut ticker = embassy_time::Ticker::every(Duration::from_hz(IMU_SAMPLE_RATE_HZ as u64));
    let mut baro_counter = 0u8;
    let mut mag_counter = 0u8;

    // Делители частоты для разных датчиков
    const BARO_DIVIDER: u8 = (IMU_SAMPLE_RATE_HZ / BARO_SAMPLE_RATE_HZ) as u8; // 100Hz / 25Hz = 4
    const MAG_DIVIDER: u8 = 5; // 100Hz / 20Hz = 5

    loop {
        ticker.next().await;
        let current_time = Instant::now();

        // Читаем IMU
        // === Чтение IMU (100 Hz) ===
        match imu.read_all(&mut i2c).await {
            Ok(imu_data) => {
                // Читаем магнитометр (реже - 20Hz)
                let mag_heading = {
                    mag_counter += 1;
                    if mag_counter >= MAG_DIVIDER {
                        mag_counter = 0;
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
                let fused_data = fusion_filter.update(
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
                    timestamp_us: current_time.as_micros(),
                };

                // Обновляем глобальное состояние
                *SYSTEM_STATE.last_imu.lock().await = Some(imu_msg);

                // Отправляем данные IMU
                if let Err(_) = CHANNELS.imu_channel.try_send(imu_msg) {
                    // defmt::trace!("Буфер IMU канала переполнен");
                    // TODO придумать что тут делать
                }

                // defmt::trace!(
                //     "IMU: roll={:?}° pitch={:?}° yaw={:?}°",
                //     fused_data.roll * 180.0 / core::f32::consts::PI,
                //     fused_data.pitch * 180.0 / core::f32::consts::PI,
                //     fused_data.yaw * 180.0 / core::f32::consts::PI
                // );
            }
            Err(e) => {
                defmt::error!("Ошибка чтения IMU: {}", e);
            }
        }

        // === Чтение барометра (25 Hz) ===
        baro_counter += 1;
        if baro_counter >= BARO_DIVIDER {
            baro_counter = 0;

            match baro.read(&mut i2c).await {
                Ok((temperature, pressure)) => {
                    // Обрабатываем данные высоты
                    let (altitude, vertical_speed) =
                        altitude_processor.update(pressure, temperature, current_time.as_micros());

                    // Формируем сообщение высоты
                    let altitude_data = AltitudeData {
                        altitude_m: altitude,
                        vertical_speed_mps: vertical_speed,
                        pressure_pa: pressure,
                        temperature_c: temperature,
                        timestamp_us: current_time.as_micros(),
                    };

                    // Отправляем данные высоты TODO пока некому
                    // if let Err(_) = CHANNELS.altitude_channel.try_send(altitude_data) {
                    //     defmt::trace!("Буфер канала высоты переполнен");
                    // }

                    // Обновляем глобальное состояние
                    *SYSTEM_STATE.last_altitude.lock().await = Some(altitude_data);

                    // defmt::trace!(
                    //     "Барометр: высота={:?}м, верт.скорость={:?}м/с, темп={:?}°C",
                    //     altitude,
                    //     vertical_speed,
                    //     temperature
                    // );
                }
                Err(e) => {
                    defmt::error!("Ошибка чтения барометра: {}", e);
                }
            }
        }

        // === Проверка состояния системы ===
        static mut ERROR_COUNT: u32 = 0;
        unsafe {
            // Если слишком много ошибок подряд, переходим в аварийный режим
            if ERROR_COUNT > 100 {
                defmt::error!("Слишком много ошибок датчиков!");
                *SYSTEM_STATE.flight_mode.lock().await = crate::data::FlightMode::Emergency;
                ERROR_COUNT = 0;
            }
        }
    }
}
