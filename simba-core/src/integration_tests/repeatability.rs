use std::path::Path;

use crate::simulator::{Record, Simulator};

macro_rules! replication_test {
    ($config:ident) => {
#[test]
fn $config() {
    let nb_replications = 10;

    let mut results: Vec<Vec<Record>> = Vec::new();

    for i in 0..nb_replications {
        print!("Run {}/{nb_replications} ... ", i + 1);
        let mut simulator =
            Simulator::from_config_path(Path::new(format!("test_config/{}.yaml", stringify!($config)).as_str()), &None).map_err(|e| {
                println!("Error while loading config: {}", e.detailed_error());
                e
            }).unwrap();

        simulator.run().unwrap();

        results.push(simulator.get_records(true));
        println!("OK");
    }

    let reference_result = &results[0];
    assert!(!reference_result.is_empty());
    for result in results.iter().skip(1) {
        assert_eq!(result.len(), reference_result.len());
        for (j, ref_result) in reference_result.iter().enumerate() {
            let result_as_str = format!("{:?}", result[j]);
            let reference_result_as_str = format!("{:?}", ref_result);
            assert_eq!(
                result_as_str, reference_result_as_str,
                "{result_as_str} != {reference_result_as_str}"
            );
        }
    }
}
    };
}

replication_test!(config);

