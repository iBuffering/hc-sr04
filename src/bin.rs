use clap::{Parser, Subcommand, ValueEnum};

#[derive(Parser, Debug)]
#[command(version, about, long_about = None)]
#[command(propagate_version = true)]
struct Cli {
    #[command(subcommand)]
    command: Commands,
}

#[derive(Debug, Subcommand)]
enum Commands {
    /// Measures the distance in the specified unit.
    Measure {
        /// The **TIRG** output pin.
        trig: u8,
        /// The **ECHO** input pin.
        echo: u8,
        /// Max range in meters.
        #[arg(short, long, default_value_t = 4.0)]
        max_range: f32,
        /// Unit of measure.
        #[arg(short, long, value_enum, default_value_t = Unit::Centimeters)]
        unit: Unit,
        /// Number of samples.
        #[arg(short, long)]
        samples: Option<usize>,
    },
}

#[derive(Debug, Copy, Clone, ValueEnum)]
enum Unit {
    Millimeters,
    Centimeters,
    Decimeters,
    Meters,
}

impl Unit {
    fn to_sensor_unit(self) -> hc_sr04::Unit {
        match self {
            Self::Millimeters => hc_sr04::Unit::Millimeters,
            Self::Centimeters => hc_sr04::Unit::Centimeters,
            Self::Decimeters => hc_sr04::Unit::Decimeters,
            Self::Meters => hc_sr04::Unit::Meters,
        }
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = Cli::parse();

    match args.command {
        Commands::Measure {
            trig,
            echo,
            max_range,
            unit,
            samples,
        } => {
            let mut sensor = hc_sr04::HcSr04::new(trig, echo, None, Some(max_range))?;
            let distance = if let Some(samples) = samples {
                sensor.measure_median(samples, unit.to_sensor_unit())?
            } else {
                sensor.measure_distance(unit.to_sensor_unit())?
            };

            match distance {
                Some(dist) => println!("{dist:.2}"),
                None => println!("None"),
            }
        }
    }

    Ok(())
}
