// hc-sr04: Raspberry Pi Rust driver for the HC-SR04 ultrasonic distance sensor.
// Copyright (C) 2022 Marco Radocchia
//
// This program is free software: you can redistribute it and/or modify it under
// the terms of the GNU General Public License as published by the Free Software
// Foundation, either version 3 of the License, or (at your option) any later
// version.
//
// This program is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
// details.
//
// You should have received a copy of the GNU General Public License along with
// this program. If not, see https://www.gnu.org/licenses/.
//
//! **HC-SR04** ultrasonic distance sensor driver.
//!
//! This crate provides a driver for the **HC-SR04**/**HC-SR04P** ultrasonic distance sensor on
//! *Raspberry Pi*, using [rppal](https://docs.rs/rppal/0.13.1/rppal/) to access Raspberry Pi's
//! GPIO.
//!
//! ## Examples
//!
//! Usage examples can be found in the
//! [examples](https://github.com/marcoradocchia/hc-sr04/tree/master/examples) folder.
//!
//! ## Measure distance
//! ```no_run
//! use hc_sr04::{HcSr04, Unit};
//!
//! // Initialize driver.
//! let mut ultrasonic = HcSr04::new(
//!     24,           // TRIGGER -> Gpio pin 24
//!     23,           // ECHO -> Gpio pin 23
//!     Some(23_f32), // Ambient temperature (if `None` defaults to 20.0C)
//!     None,         // Max range (if `None` defaults to 4m)
//! ).unwrap();
//!
//! // Perform distance measurement, specifying measuring unit of return value.
//! match ultrasonic.measure_distance(Unit::Meters).unwrap() {
//!     Some(dist) => println!("Distance: {dist:.2}m"),
//!     None => println!("Object out of range"),
//! }
//! ```
//!
//! ## Calibrate measurement
//!
//! Distance measurement can be calibrated at runtime using the [`HcSr04::calibrate`] method that
//! this library exposes, passing the current ambient temperature as `f32`.
//!
//! ```no_run
//! use hc_sr04::{HcSr04, Unit};
//!
//! // Initialize driver.
//! let mut ultrasonic = HcSr04::new(24, 23, None, None).unwrap();
//!
//! // Calibrate measurement with ambient temperature.
//! ultrasonic.calibrate(23_f32);
//!
//! // Perform distance measurement.
//! match ultrasonic.measure_distance(Unit::Centimeters).unwrap() {
//!     Some(dist) => println!("Distance: {dist:.1}cm"),
//!     None => println!("Object out of range"),
//! }
//! ```

pub mod error;

use error::Error;
use rppal::gpio::{Gpio, InputPin, OutputPin, Trigger};
use std::{
    thread,
    time::{Duration, Instant},
};

pub type Result<T> = std::result::Result<T, Error>;

/// Measuring unit (defaults to [`Unit::Meters`]).
pub enum Unit {
    Millimeters,
    Centimeters,
    Decimeters,
    Meters,
}

/// **HC-SR04** ultrasonic sensor on *Raspberry Pi*.
#[derive(Debug)]
pub struct HcSr04 {
    /// **TRIGGER** output GPIO pin.
    trig: OutputPin,
    /// **ECHO** input GPIO pin.
    echo: InputPin,
    /// speed of sound given the ambient **Temperature**.
    sound_speed: f32,
    /// **ECHO** pin `FallingEdge` polling timeout, considering
    /// the maximum measuring range for the sensor and the speed of sound
    /// given the ambient **Temperature**.
    timeout: Duration,
    /// Sensor max range in meters.
    max_range: f32,
}

impl HcSr04 {
    /// Perform `sound_speed` and `timeout` calculations required to calibrate the sensor,
    /// based on **ambient temperature** `temp` and `max_range` distance.
    fn calibration_calc(max_range: f32, temp: f32) -> (f32, Duration) {
        /// Speed of sound at 0C in m/s.
        const SOUND_SPEED_0C: f32 = 331.3;
        /// Increase speed of sound over temperature factor m/[sC].
        const SOUND_SPEED_INC_OVER_TEMP: f32 = 0.606;

        // Speed of sound, depending on ambient temperature (if `temp` is `None`, default to 20C).
        let sound_speed = SOUND_SPEED_0C + (SOUND_SPEED_INC_OVER_TEMP * temp);

        // Polling timeout for **ECHO** pin: since max range for HC-SR04 is 4m, it doesn't make
        // sense to wait longer than the time required to the ultrasonic sound wave to cover the
        // max range distance. In other words, if the timeout is reached, the measurement was not
        // successfull or the object is located too far away from the sensor in order to be
        // detected.
        let timeout = Duration::from_secs_f32(max_range / sound_speed * 2.);

        (sound_speed, timeout)
    }

    /// Initialize HC-SR04 sensor and register GPIO interrupt on `echo` pin for `RisingEdge` events
    /// in order to poll it for bouncing ultrasonic waves detection.
    ///
    /// # Parameters
    ///
    /// - `trig`: **TRIGGER** output GPIO pin
    /// - `echo`: **ECHO** input GPIO pin
    /// - `temp`: ambient **TEMPERATURE** used for calibration (if `None` defaults to `20.0`)
    /// - `max_range`: Max range in meters (if `None` defaults to `4m`)
    ///
    /// # Errors
    ///
    /// Returns an `Error(Gpio::Error)` when failing to interface with the GPIO
    /// peripheral.
    pub fn new(trig: u8, echo: u8, temp: Option<f32>, max_range: Option<f32>) -> Result<Self> {
        let max_range = max_range.unwrap_or(4.0);

        let gpio = Gpio::new()?;

        let mut echo = gpio.get(echo)?.into_input_pulldown();
        echo.set_interrupt(Trigger::Both, None)?;

        let (sound_speed, timeout) = Self::calibration_calc(max_range, temp.unwrap_or(20.));

        Ok(Self {
            trig: gpio.get(trig)?.into_output_low(),
            echo,
            sound_speed,
            timeout,
            max_range,
        })
    }

    /// Calibrate the sensor with the given **ambient temperature** (`temp`) expressed as *Celsius
    /// degrees*.
    pub fn calibrate(&mut self, temp: f32) {
        (self.sound_speed, self.timeout) = Self::calibration_calc(self.max_range, temp);
    }

    /// Perform a **distance measurement**.
    ///
    /// Returns the measured distance in the provided `unit` of measurement,
    /// or `None` if the measurement exceeds the max range set.
    ///
    /// # Errors
    ///
    /// Returns `Error::Gpio(Gpio::Error)` when failing to interface with the GPIO
    /// peripheral.
    /// Returns `Error::SensorNotConnected` when failing to communicate with the sensor.
    #[allow(clippy::needless_pass_by_value)]
    pub fn measure_distance(&mut self, unit: Unit) -> Result<Option<f32>> {
        let Some(rtt) = self.measure_rtt()? else {
            return Ok(None);
        };

        // Distance in m.
        let distance = (self.sound_speed * rtt) / 2.;

        Ok(Some(match unit {
            Unit::Millimeters => distance * 1000.,
            Unit::Centimeters => distance * 100.,
            Unit::Decimeters => distance * 10.,
            Unit::Meters => distance,
        }))
    }

    /// Performs a measurement.
    ///
    /// Returns the round trip time of the ultrasonic wave in seconds, or `None`
    /// if the measurement exceeds the max range set.
    ///
    /// See also [`measure_distance`](HcSr04::measure_distance).
    ///
    /// # Errors
    ///
    /// Returns `Error::Gpio(Gpio::Error)` when failing to interface with the GPIO
    /// peripheral.
    /// Returns `Error::SensorNotConnected` when failing to communicate with the sensor.
    pub fn measure_rtt(&mut self) -> Result<Option<f32>> {
        // Poll for interrupts, clearing all cached events, with a timeout of zero.
        // Effectively, this clears all cached events and immediately returns.
        self.echo.poll_interrupt(true, Some(Duration::ZERO))?;

        self.trig.set_high();
        thread::sleep(Duration::from_micros(10));
        self.trig.set_low();

        // Wait for the `RisingEdge` event.
        // If timeout is reached three times in a row, the sensor is not connected.
        //
        // NOTE: After 1000 samples collected measuring the time it takes for our HC-SR04
        // to raise the ECHO signal, calculating the P99 value and adding 100us as
        // a safety margin, the most suitable timeout value was 570us.
        // Unfortunately `libc`, and in turn `rppal`, accept a timeout in milliseconds.
        // This means that a timeout with a lower precision gets interpreted as 0,
        // thus rendering all calculations useless.
        // A timeout of 1ms was chosen because it's the smallest available, and
        // in our tests the interval rarely ever went above 800us.
        // To be safe, the program tries three times before signalling an issue.
        //
        // See "https://github.com/golemparts/rppal/blob/b371a7a548364455e9a54ed526a435592100e0a1/src/gpio/epoll.rs#L98-L102"
        let mut tries = 0;
        loop {
            if let Some(event) = self
                .echo
                .poll_interrupt(false, Some(Duration::from_millis(1)))?
                && event.trigger == Trigger::RisingEdge
            {
                break;
            }

            tries += 1;
            if tries >= 3 {
                return Err(Error::SensorNotConnected);
            }
        }

        let instant = Instant::now();

        // Wait for the `FallingEdge`.
        // If timeout is reached, the object is outside of the sensor's range.
        if self
            .echo
            .poll_interrupt(false, Some(self.timeout))?
            .is_none()
        {
            return Ok(None);
        }

        // Return elapsed time in seconds.
        Ok(Some(instant.elapsed().as_secs_f32()))
    }
}
