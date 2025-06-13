pub trait ToVec<T> {
    fn to_vec() -> Vec<T>;
}

pub trait FromString
where
    Self: Sized,
{
    fn from_string(str: &String) -> Option<Self>;
}
