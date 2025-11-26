use std::collections::BTreeMap;

use libm::ceilf;

#[derive(Debug)]
pub(super) struct TimeAnalysisStatistics {
    pub mean: f32,
    pub median: f32,
    pub min: f32,
    pub max: f32,
    pub n: u32,
    pub q1: f32,
    pub q3: f32,
    pub q99: f32,
    pub q01: f32,
}

impl TimeAnalysisStatistics {
    pub fn from_array(mut v: Vec<f32>) -> Self {
        v.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let sum: f32 = v.iter().sum();
        let n = v.len();
        let nf32 = n as f32;
        TimeAnalysisStatistics {
            mean: sum / nf32,
            median: if n.is_multiple_of(2) {
                (v[n / 2] + v[n / 2 - 1]) / 2.
            } else {
                v[(n - 1) / 2]
            },
            max: *v
                .iter()
                .max_by(|a, b| a.partial_cmp(b).unwrap())
                .unwrap_or(&f32::INFINITY),
            min: *v
                .iter()
                .min_by(|a, b| a.partial_cmp(b).unwrap())
                .unwrap_or(&0.),
            n: (n as u32),
            q1: v[(ceilf(nf32 / 4.) as usize).min(n - 1)],
            q3: v[(ceilf(nf32 * 0.75) as usize).min(n - 1)],
            q01: v[(ceilf(nf32 * 0.01) as usize).min(n - 1)],
            q99: v[(ceilf(nf32 * 0.99) as usize).min(n - 1)],
        }
    }

    pub fn as_map(&self) -> BTreeMap<String, String> {
        let mut map = BTreeMap::<String, String>::new();
        map.insert("mean".to_string(), self.mean.to_string());
        map.insert("median".to_string(), self.median.to_string());
        map.insert("min".to_string(), self.min.to_string());
        map.insert("max".to_string(), self.max.to_string());
        map.insert("n".to_string(), self.n.to_string());
        map.insert("q1".to_string(), self.q1.to_string());
        map.insert("q3".to_string(), self.q3.to_string());
        map.insert("q99".to_string(), self.q99.to_string());
        map.insert("q01".to_string(), self.q01.to_string());
        map
    }
}
