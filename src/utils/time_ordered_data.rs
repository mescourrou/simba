/*!
Module providing a data structure to store data with a timestamp, with
increasing order.
*/

use core::slice::{Iter, IterMut};
use std::iter::Skip;
use std::vec::Vec;

/// Data structure to store ordered timed data.
///
/// The generic is the Type to be stored. For now, the time is stored
/// using `f32` without possibility of modifying it.
///
/// ## Example
/// ```
/// use simba::utils::time_ordered_data::TimeOrderedData;
///
/// let mut tod = TimeOrderedData::<String>::new();
/// tod.insert(2.1, String::from("Hello1"), true);
/// tod.insert(2.9, String::from("Hello2"), true);
/// tod.insert(2.6, String::from("Hello3"), true);
///
/// for data in tod.iter_from_time(2.2) {
///     println!("Time {}: {}", data.0, data.1);
/// }
/// ```
///
/// Output:
/// ```txt
/// Time 2.6: Hello3
/// Time 2.9: Hello2
/// ```
///
#[derive(Debug, Clone)]
pub struct TimeOrderedData<T> {
    /// Data structure. WARNING: the sort is done during the insertion,
    /// and is not checked after !
    data: Vec<(f32, T)>,
}

impl<T> TimeOrderedData<T> {
    /// Creates a new empty data structure.
    pub fn new() -> Self {
        Self { data: Vec::new() }
    }

    /// Find the index equal or just after the required time.
    ///
    /// This function was primarly made for insertion. So the position returned
    /// is the position where to insert a new element.
    ///
    /// ## Arguments
    /// * `time` -- timestamp looked for.
    ///
    /// ## Return
    /// * `usize` -- position of the element with equal time or just after.
    /// * `bool` -- Is the element found is equal (the position is then this element).
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

    /// Insert an element at the given time.
    ///
    /// ## Arguments
    /// * `time` -- Time where to insert the new element.
    /// * `data` -- Element to insert.
    /// * `do_erase` -- Erase or not if an element is already at the same timestamp.
    /// If it does not erase, multiple elements will have the same timestamp.
    ///
    /// TODO: test without erase, if all elements come out by iter.
    pub fn insert(&mut self, time: f32, data: T, do_erase: bool) {
        let (pos, erase) = self.find_time_position(time);

        if erase {
            if do_erase {
                self.data[pos] = (time, data);
            } else {
                self.data.insert(pos + 1, (time, data));
            }
        } else {
            self.data.insert(pos, (time, data));
        }
    }

    /// Get a mutable reference on the data just before or at the given `time`.
    ///
    /// ## Return
    /// Return an Option with:
    /// * `Some(time, mutable reference on data)` if a data was found.
    /// * `None` if no data was found, e.g. if `time` is below the minimal time.
    pub fn get_data_beq_time_mut(&mut self, mut time: f32) -> Option<(f32, &mut T)> {
        for (data_time, ref mut data) in self.data.iter_mut().rev() {
            if data_time <= &mut time {
                return Some((time, data));
            }
        }
        None
    }

    /// Get a reference on the data just before or at the given `time`.
    ///
    /// ## Return
    /// Return an Option with:
    /// * `Some(time, reference on data)` if a data was found.
    /// * `None` if no data was found, e.g. if `time` is below the minimal time.
    pub fn get_data_beq_time(&self, time: f32) -> Option<(f32, &T)> {
        for (data_time, data) in self.data.iter().rev() {
            if data_time <= &time {
                return Some((time, data));
            }
        }
        None
    }

    /// Get a mutable reference on the data strictly before the given `time`.
    ///
    /// ## Return
    /// Return an Option with:
    /// * `Some(time, mutable reference on data)` if a data was found.
    /// * `None` if no data was found, e.g. if `time` is below the minimal time.
    pub fn get_data_before_time_mut(&mut self, mut time: f32) -> Option<(f32, &mut T)> {
        for (data_time, ref mut data) in self.data.iter_mut().rev() {
            if data_time < &mut time {
                return Some((time, data));
            }
        }
        None
    }

    /// Get a reference on the data strictly before the given `time`.
    ///
    /// ## Return
    /// Return an Option with:
    /// * `Some(time, reference on data)` if a data was found.
    /// * `None` if no data was found, e.g. if `time` is below the minimal time.
    pub fn get_data_before_time(&self, time: f32) -> Option<(f32, &T)> {
        for (data_time, data) in self.data.iter().rev() {
            if data_time < &time {
                return Some((time, data));
            }
        }
        None
    }

    /// Get a mutable reference on the data just after or at the given `time`.
    ///
    /// ## Return
    /// Return an Option with:
    /// * `Some(time, mutable reference on data)` if a data was found.
    /// * `None` if no data was found, e.g. if `time` is after the maximal time.
    pub fn get_data_geq_time_mut(&mut self, mut time: f32) -> Option<(f32, &mut T)> {
        for (data_time, ref mut data) in self.data.iter_mut() {
            if data_time >= &mut time {
                return Some((time, data));
            }
        }
        None
    }

    /// Get a reference on the data just after or at the given `time`.
    ///
    /// ## Return
    /// Return an Option with:
    /// * `Some(time, reference on data)` if a data was found.
    /// * `None` if no data was found, e.g. if `time` is after the maximal time.
    pub fn get_data_geq_time(&self, time: f32) -> Option<(f32, &T)> {
        for (data_time, data) in self.data.iter() {
            if data_time >= &time {
                return Some((time, data));
            }
        }
        None
    }

    /// Get a mutable reference on the data strictly after the given `time`.
    ///
    /// ## Return
    /// Return an Option with:
    /// * `Some(time, mutable reference on data)` if a data was found.
    /// * `None` if no data was found, e.g. if `time` is after the maximal time.
    pub fn get_data_after_time_mut(&mut self, mut time: f32) -> Option<(f32, &mut T)> {
        for (data_time, ref mut data) in self.data.iter_mut() {
            if data_time > &mut time {
                return Some((time, data));
            }
        }
        None
    }

    /// Get a reference on the data strictly after the given `time`.
    ///
    /// ## Return
    /// Return an Option with:
    /// * `Some(time, reference on data)` if a data was found.
    /// * `None` if no data was found, e.g. if `time` is after the maximal time.
    pub fn get_data_after_time(&self, time: f32) -> Option<(f32, &T)> {
        for (data_time, data) in self.data.iter() {
            if data_time > &time {
                return Some((time, data));
            }
        }
        None
    }

    /// Get a reference on the data strictly (within 1e-15) at the given `time`.
    ///
    /// ## Return
    /// Return an Option with:
    /// * `Some(time, reference on data)` if a data was found.
    /// * `None` if no data was found at this `time`.
    pub fn get_data_at_time(&self, time: f32) -> Option<(f32, &T)> {
        for (data_time, data) in self.data.iter() {
            if (data_time - &time).abs() < 1e-15 {
                return Some((time, data));
            }
        }
        None
    }

    /// Get a mutable reference on the data strictly (within 1e-15) at the given `time`.
    ///
    /// ## Return
    /// Return an Option with:
    /// * `Some(time, mutable reference on data)` if a data was found.
    /// * `None` if no data was found at this `time`.
    pub fn get_data_at_time_mut(&mut self, time: f32) -> Option<(f32, &mut T)> {
        for (data_time, ref mut data) in self.data.iter_mut() {
            if (*data_time - time).abs() < 1e-15 {
                return Some((time, data));
            }
        }
        None
    }

    /// Provide an iterator from the given `time`, in the chronological order.
    ///
    /// If `time` is an existent time, the iterator starts at this position.
    pub fn iter_from_time(&self, time: f32) -> Skip<Iter<'_, (f32, T)>> {
        let (mut pos, _) = self.find_time_position(time);
        while pos > 0 && (self.data[pos - 1].0 - time).abs() < 1e-15 {
            pos -= 1;
        }
        self.data.iter().skip(pos)
    }

    /// Provide an mutable iterator from the given `time`, in the chronological order.
    ///
    /// If `time` is an existent time, the iterator starts at this position.
    pub fn iter_from_time_mut(&mut self, time: f32) -> Skip<IterMut<'_, (f32, T)>> {
        let (pos, _) = self.find_time_position(time);
        self.data.iter_mut().skip(pos)
    }

    /// Provide an iterator which goes on all the data in the chronological order.
    pub fn iter(&self) -> impl Iterator<Item = &(f32, T)> {
        self.data.iter()
    }

    /// Remove a data at the given `time` and returns it.
    ///
    /// If there is no data at the given time, `None` is returned.
    pub fn remove(&mut self, time: f32) -> Option<(f32, T)> {
        let (pos, erase) = self.find_time_position(time);
        if !erase {
            return None;
        }

        Some(self.data.remove(pos))
    }

    /// Size of the data structure.
    pub fn len(&self) -> usize {
        self.data.len()
    }

    /// Minimal time in the data structure.
    pub fn min_time(&self) -> Option<f32> {
        if self.len() == 0 {
            None
        } else {
            Some(self.data[0].0)
        }
    }

    /// Maximal time in the data structure.
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

    #[test]
    fn do_not_erase() {
        let mut tod = TimeOrderedData::<String>::new();
        let str_to_insert = String::from("Hello1");
        tod.insert(2.1, str_to_insert, false);
        let str_to_insert = String::from("Hello2");
        tod.insert(2.6, str_to_insert, false);
        let str_to_insert = String::from("Hello3");
        tod.insert(2.6, str_to_insert, false);

        let mut iterator = tod.iter_from_time(2.6);
        let tpl = iterator.next().unwrap();
        assert_eq!(tpl.0, 2.6);
        assert_eq!(tpl.1, String::from("Hello2"));
        assert_eq!(
            iterator.next(),
            Some((2.6, String::from("Hello3"))).as_ref()
        );
        assert_eq!(iterator.next(), None);
    }
}
