use clap::{Parser, Subcommand};

#[derive(Parser)]
#[command(name = "trilink", about = "Tri-Link integration bridge")]
struct Cli {
    #[command(subcommand)]
    command: Commands,
}

#[derive(Subcommand)]
enum Commands {
    /// Run the full pipeline with a configuration file.
    Run {
        #[arg(long, default_value = "config.toml")]
        config: String,
    },
}

fn main() {
    let cli = Cli::parse();
    match cli.command {
        Commands::Run { config } => {
            eprintln!("tri-link run — config: {config} (full pipeline not yet implemented)");
        }
    }
}
