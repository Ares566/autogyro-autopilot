//! Модуль алгоритмов управления

// pub mod pid;
pub mod attitude;
mod fuzzy;
// pub mod altitude;
// pub mod position;

// Реэкспорт основных типов
pub use attitude::AttitudeController;
// pub use pid::PidController;
