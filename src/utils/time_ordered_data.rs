use core::slice::{Iter, IterMut};
use std::iter::Skip;
use std::vec::Vec;

#[derive(Debug, Clone)]
pub struct TimeOrderedData<T> {
    data: Vec<(f32, T)>,
}

impl<T> TimeOrderedData<T> {
    pub fn new() -> Self {
        Self { data: Vec::new() }
    }

    fn find_time_position(&self, time: f32) -> (usize, bool) {
        let mut pos = self.data.len();

        while pos > 0 {
            let pos_time = self.data[pos - 1].0;
            if pos_time < time {
                // Return.1 is erase value
                return (pos, false);
            } else if (pos_time - time).abs() < 10e-15 {
                pos = pos - 1;
                // Return.1 is erase value
                return (pos, true);
            }
            pos = pos - 1;
        }
        return (pos, false);
    }

    pub fn insert(&mut self, time: f32, data: T, do_erase: bool) {
        let (pos, erase) = self.find_time_position(time);

        if erase && do_erase {
            self.data[pos] = (time, data);
        } else {
            self.data.insert(pos, (time, data));
        }
    }

    pub fn get_data_beq_time_mut(&mut self, mut time: f32) -> Option<(f32, &mut T)> {
        for (data_time, ref mut data) in self.data.iter_mut().rev() {
            if data_time <= &mut time {
                return Some((time, data));
            }
        }
        None
    }

    pub fn get_data_beq_time(&self, time: f32) -> Option<(f32, &T)> {
        for (data_time, data) in self.data.iter().rev() {
            if data_time <= &time {
                return Some((time, data));
            }
        }
        None
    }

    pub fn get_data_before_time_mut(&mut self, mut time: f32) -> Option<(f32, &mut T)> {
        for (data_time, ref mut data) in self.data.iter_mut().rev() {
            if data_time < &mut time {
                return Some((time, data));
            }
        }
        None
    }

    pub fn get_data_before_time(&self, time: f32) -> Option<(f32, &T)> {
        for (data_time, data) in self.data.iter().rev() {
            if data_time < &time {
                return Some((time, data));
            }
        }
        None
    }

    pub fn get_data_geq_time_mut(&mut self, mut time: f32) -> Option<(f32, &mut T)> {
        for (data_time, ref mut data) in self.data.iter_mut() {
            if data_time >= &mut time {
                return Some((time, data));
            }
        }
        None
    }

    pub fn get_data_geq_time(&self, time: f32) -> Option<(f32, &T)> {
        for (data_time, data) in self.data.iter() {
            if data_time >= &time {
                return Some((time, data));
            }
        }
        None
    }

    pub fn get_data_after_time_mut(&mut self, mut time: f32) -> Option<(f32, &mut T)> {
        for (data_time, ref mut data) in self.data.iter_mut() {
            if data_time > &mut time {
                return Some((time, data));
            }
        }
        None
    }

    pub fn get_data_after_time(&self, time: f32) -> Option<(f32, &T)> {
        for (data_time, data) in self.data.iter() {
            if data_time > &time {
                return Some((time, data));
            }
        }
        None
    }

    pub fn get_data_at_time(&self, time: f32) -> Option<(f32, &T)> {
        for (data_time, data) in self.data.iter() {
            if (data_time - &time).abs() < 1e-15 {
                return Some((time, data));
            }
        }
        None
    }

    pub fn get_data_at_time_mut(&mut self, time: f32) -> Option<(f32, &mut T)> {
        for (data_time, ref mut data) in self.data.iter_mut() {
            if (*data_time - time).abs() < 1e-15 {
                return Some((time, data));
            }
        }
        None
    }

    pub fn iter_from_time(&self, time: f32) -> Skip<Iter<'_, (f32, T)>> {
        let (pos, _) = self.find_time_position(time);
        self.data.iter().skip(pos)
    }

    pub fn iter_from_time_mut(&mut self, time: f32) -> Skip<IterMut<'_, (f32, T)>> {
        let (pos, _) = self.find_time_position(time);
        self.data.iter_mut().skip(pos)
    }

    pub fn iter(&self) -> impl Iterator<Item = &(f32, T)> {
        self.data.iter()
    }

    pub fn remove(&mut self, time: f32) -> Option<(f32, T)> {
        let (pos, erase) = self.find_time_position(time);
        if !erase {
            return None;
        }

        Some(self.data.remove(pos))
    }

    pub fn len(&self) -> usize {
        self.data.len()
    }

    pub fn min_time(&self) -> Option<f32> {
        if self.len() == 0 {
            None
        } else {
            Some(self.data[0].0)
        }
    }

    pub fn max_time(&self) -> Option<f32> {
        if self.len() == 0 {
            None
        } else {
            Some(self.data[self.len() - 1].0)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::TimeOrderedData;

    #[test]
    fn new() {
        let tod = TimeOrderedData::<String>::new();
        assert_eq!(tod.data.len(), 0);
    }

    #[test]
    fn insert_when_empty() {
        let mut tod = TimeOrderedData::<String>::new();

        let str_to_insert = String::from("Hello");

        tod.insert(2.1, str_to_insert, true);

        assert_eq!(tod.data.len(), 1);
        let data = &tod.data[0];
        assert_eq!(data.0, 2.1);
        assert_eq!(data.1, String::from("Hello"));
    }

    #[test]
    fn insert_larger_time() {
        let mut tod = TimeOrderedData::<String>::new();

        // Empty
        let str_to_insert = String::from("Hello1");
        tod.insert(2.1, str_to_insert, true);

        // Insert after
        let str_to_insert = String::from("Hello2");
        tod.insert(2.5, str_to_insert, true);

        assert_eq!(tod.data.len(), 2);
        let data = &tod.data[1];
        assert_eq!(data.0, 2.5);
        assert_eq!(data.1, String::from("Hello2"));
    }

    #[test]
    fn insert_first_position() {
        let mut tod = TimeOrderedData::<String>::new();

        // Empty
        let str_to_insert = String::from("Hello1");
        tod.insert(2.1, str_to_insert, true);

        // Insert in 1st position
        let str_to_insert = String::from("Hello2");
        tod.insert(1.5, str_to_insert, true);

        assert_eq!(tod.data.len(), 2);
        let data = &tod.data[0];
        assert_eq!(data.0, 1.5);
        assert_eq!(data.1, String::from("Hello2"));
    }

    #[test]
    fn insert_in_between() {
        let mut tod = TimeOrderedData::<String>::new();

        // Empty
        let str_to_insert = String::from("Hello1");
        tod.insert(2.1, str_to_insert, true);

        // Insert after
        let str_to_insert = String::from("Hello2");
        tod.insert(2.6, str_to_insert, true);

        // Insert in between
        let str_to_insert = String::from("Hello3");
        tod.insert(2.4, str_to_insert, true);

        assert_eq!(tod.data.len(), 3);
        // 0
        let data = &tod.data[0];
        assert_eq!(data.0, 2.1);
        assert_eq!(data.1, String::from("Hello1"));

        // 1
        let data = &tod.data[1];
        assert_eq!(data.0, 2.4);
        assert_eq!(data.1, String::from("Hello3"));

        // 2
        let data = &tod.data[2];
        assert_eq!(data.0, 2.6);
        assert_eq!(data.1, String::from("Hello2"));
    }

    #[test]
    fn insert_with_replacement() {
        let mut tod = TimeOrderedData::<String>::new();

        // Empty
        let str_to_insert = String::from("Hello1");
        tod.insert(2.1, str_to_insert, true);

        // Insert at same place
        let str_to_insert = String::from("Hello2");
        tod.insert(2.1, str_to_insert, true);

        assert_eq!(tod.data.len(), 1);
        let data = &tod.data[0];
        assert_eq!(data.0, 2.1);
        assert_eq!(data.1, String::from("Hello2"));
    }

    #[test]
    fn get_data_before_and_equal_time() {
        let mut tod = TimeOrderedData::<String>::new();
        tod.insert(2.1, String::from("Hello"), true);
        tod.insert(2.3, String::from("Hello2"), true);

        assert_eq!(
            *tod.get_data_beq_time(2.2).unwrap().1,
            String::from("Hello")
        );
        assert_eq!(
            *tod.get_data_beq_time(2.1).unwrap().1,
            String::from("Hello")
        );
        assert_eq!(tod.get_data_beq_time(2.), None);
        assert_eq!(tod.get_data_before_time(2.1), None);
    }

    #[test]
    fn get_data_before_and_equal_time_mut() {
        let mut tod = TimeOrderedData::<String>::new();
        tod.insert(2.1, String::from("Hello"), true);
        tod.insert(2.3, String::from("Hello2"), true);

        assert_eq!(
            *tod.get_data_beq_time(2.2).unwrap().1,
            String::from("Hello")
        );
        let data: &mut String = tod.get_data_beq_time_mut(2.2).unwrap().1;
        *data = String::from("Hello1");
        assert_eq!(
            *tod.get_data_beq_time(2.2).unwrap().1,
            String::from("Hello1")
        );
        assert_eq!(
            *tod.get_data_beq_time(2.1).unwrap().1,
            String::from("Hello1")
        );
        assert_eq!(tod.get_data_beq_time(2.), None);

        assert_eq!(tod.get_data_before_time_mut(2.1), None);
    }

    #[test]
    fn get_data_after_and_equal_time() {
        let mut tod = TimeOrderedData::<String>::new();
        tod.insert(2.1, String::from("Hello"), true);
        tod.insert(2.3, String::from("Hello2"), true);

        assert_eq!(
            *tod.get_data_geq_time(2.2).unwrap().1,
            String::from("Hello2")
        );
        assert_eq!(
            *tod.get_data_geq_time(2.1).unwrap().1,
            String::from("Hello")
        );
        assert_eq!(*tod.get_data_geq_time(2.).unwrap().1, String::from("Hello"));
        assert_eq!(tod.get_data_geq_time(2.4), None);
        assert_eq!(tod.get_data_after_time(2.3), None);
    }

    #[test]
    fn get_data_after_and_equal_time_mut() {
        let mut tod = TimeOrderedData::<String>::new();
        tod.insert(2.1, String::from("Hello"), true);
        tod.insert(2.3, String::from("Hello2"), true);

        assert_eq!(
            *tod.get_data_geq_time(2.2).unwrap().1,
            String::from("Hello2")
        );
        let data: &mut String = tod.get_data_geq_time_mut(2.2).unwrap().1;
        *data = String::from("Hello1");
        assert_eq!(
            *tod.get_data_geq_time(2.2).unwrap().1,
            String::from("Hello1")
        );
        assert_eq!(
            *tod.get_data_geq_time(2.1).unwrap().1,
            String::from("Hello")
        );
        assert_eq!(tod.get_data_geq_time(2.4), None);

        assert_eq!(tod.get_data_after_time_mut(2.3), None);
    }

    #[test]
    fn iter_from_time() {
        let mut tod = TimeOrderedData::<String>::new();
        let str_to_insert = String::from("Hello1");
        tod.insert(2.1, str_to_insert, true);
        let str_to_insert = String::from("Hello2");
        tod.insert(2.6, str_to_insert, true);
        let str_to_insert = String::from("Hello3");
        tod.insert(2.9, str_to_insert, true);

        let mut iterator = tod.iter_from_time(2.2);
        assert_eq!(
            iterator.next(),
            Some((2.6, String::from("Hello2"))).as_ref()
        );
        assert_eq!(
            iterator.next(),
            Some((2.9, String::from("Hello3"))).as_ref()
        );
        assert_eq!(iterator.next(), None);
    }
    #[test]
    fn iter_from_time_mut() {
        let mut tod = TimeOrderedData::<String>::new();
        let str_to_insert = String::from("Hello1");
        tod.insert(2.1, str_to_insert, true);
        let str_to_insert = String::from("Hello2");
        tod.insert(2.6, str_to_insert, true);
        let str_to_insert = String::from("Hello3");
        tod.insert(2.9, str_to_insert, true);

        let mut iterator = tod.iter_from_time_mut(2.2);
        let tpl = &mut iterator.next().unwrap();
        assert_eq!(tpl.0, 2.6);
        assert_eq!(tpl.1, String::from("Hello2"));
        tpl.1 = String::from("Hello");
        assert_eq!(
            iterator.next(),
            Some((2.9, String::from("Hello3"))).as_mut()
        );
        assert_eq!(iterator.next(), None);

        // Test if modification occurs
        assert_eq!(tod.data[1].1, String::from("Hello"));
    }

    #[test]
    fn remove_element() {
        let mut tod = TimeOrderedData::<String>::new();
        let str_to_insert = String::from("Hello1");
        tod.insert(2.1, str_to_insert, true);
        let str_to_insert = String::from("Hello2");
        tod.insert(2.6, str_to_insert, true);
        let str_to_insert = String::from("Hello3");
        tod.insert(2.9, str_to_insert, true);

        assert_eq!(tod.len(), 3);

        let (time, data) = tod.remove(2.1).unwrap();
        assert_eq!(time, 2.1);
        assert_eq!(data, String::from("Hello1"));

        assert_eq!(tod.len(), 2);
    }
}
