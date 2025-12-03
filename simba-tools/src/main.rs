use std::fs;
use std::collections::BTreeSet;

use clap::Parser;

#[derive(Parser)]
#[command(version, about)]
struct Cli {
    #[arg(long)]
    generate_schema: Option<String>,
}

fn generate_schema(path: String) {
    use schemars::schema_for;
    use simba::simulator::SimulatorConfig;

    let schema = schema_for!(SimulatorConfig);
    let json = serde_json::to_string_pretty(&schema).unwrap();
    fs::write(&path, json).unwrap();
    println!("Schema generated at: {}", path);
}


fn main() {
    let args = Cli::parse();

    if let Some(schema_path) = args.generate_schema {
        generate_schema(schema_path);
    }

}
