/// Landmark struct, with an `id` and a `pose`, used to read the map file.
#[derive(Debug, Clone)]
pub struct OrientedLandmark {
    pub id: i32,
    pub labels: Vec<String>,
    pub pose: Vector3<f32>,
    /// Height of the landmark, used for obstruction checks
    /// Use 0 for fully transparent landmarks
    pub height: f32,
    /// Can be 0 for ponctual landmarks
    pub width: f32,
}

use std::fmt;

use nalgebra::Vector3;
use serde::ser::{Serialize, SerializeStruct, Serializer};

impl Serialize for OrientedLandmark {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        // 3 is the number of fields in the struct.
        let mut state = serializer.serialize_struct("OrientedLandmark", 7)?;
        state.serialize_field("id", &self.id)?;
        state.serialize_field("labels", &self.labels)?;
        state.serialize_field("x", &self.pose.x)?;
        state.serialize_field("y", &self.pose.y)?;
        state.serialize_field("theta", &self.pose.z)?;
        state.serialize_field("height", &self.height)?;
        state.serialize_field("width", &self.width)?;
        state.end()
    }
}

use serde::de::{self, Deserialize, Deserializer, MapAccess, SeqAccess, Visitor};

impl<'de> Deserialize<'de> for OrientedLandmark {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        enum Field {
            Id,
            Labels,
            X,
            Y,
            Theta,
            Height,
            Width,
            Unknown,
        }

        // This part could also be generated independently by:
        //
        //    #[derive(Deserialize)]
        //    #[serde(field_identifier, rename_all = "lowercase")]
        //    enum Field { Secs, Nanos }
        impl<'de> Deserialize<'de> for Field {
            fn deserialize<D>(deserializer: D) -> Result<Field, D::Error>
            where
                D: Deserializer<'de>,
            {
                struct FieldVisitor;

                impl<'de> Visitor<'de> for FieldVisitor {
                    type Value = Field;

                    fn expecting(&self, formatter: &mut fmt::Formatter) -> fmt::Result {
                        formatter.write_str(
                            "`id` or `labels` or `x` or `y` or `theta` or `height` or `width`",
                        )
                    }

                    fn visit_str<E>(self, value: &str) -> Result<Field, E>
                    where
                        E: de::Error,
                    {
                        match value {
                            "id" => Ok(Field::Id),
                            "labels" => Ok(Field::Labels),
                            "x" => Ok(Field::X),
                            "y" => Ok(Field::Y),
                            "theta" => Ok(Field::Theta),
                            "height" => Ok(Field::Height),
                            "width" => Ok(Field::Width),
                            _ => Ok(Field::Unknown),
                        }
                    }
                }

                deserializer.deserialize_identifier(FieldVisitor)
            }
        }

        struct OrientedLandmarkVisitor;

        impl<'de> Visitor<'de> for OrientedLandmarkVisitor {
            type Value = OrientedLandmark;

            fn expecting(&self, formatter: &mut fmt::Formatter) -> fmt::Result {
                formatter.write_str("struct OrientedLandmark")
            }

            fn visit_seq<V>(self, mut seq: V) -> Result<OrientedLandmark, V::Error>
            where
                V: SeqAccess<'de>,
            {
                let id: i32 = seq
                    .next_element()?
                    .ok_or_else(|| de::Error::invalid_length(0, &self))?;
                let labels: Vec<String> = seq
                    .next_element()?
                    .ok_or_else(|| de::Error::invalid_length(1, &self))?;
                let x: f32 = seq
                    .next_element()?
                    .ok_or_else(|| de::Error::invalid_length(2, &self))?;
                let y: f32 = seq
                    .next_element()?
                    .ok_or_else(|| de::Error::invalid_length(3, &self))?;
                let theta: f32 = seq
                    .next_element()?
                    .ok_or_else(|| de::Error::invalid_length(4, &self))?;
                let height: f32 = seq
                    .next_element()?
                    .ok_or_else(|| de::Error::invalid_length(5, &self))?;
                let width: f32 = seq
                    .next_element()?
                    .ok_or_else(|| de::Error::invalid_length(6, &self))?;
                Ok(OrientedLandmark {
                    id,
                    labels,
                    pose: Vector3::from_vec(vec![x, y, theta]),
                    height,
                    width,
                })
            }

            fn visit_map<V>(self, mut map: V) -> Result<OrientedLandmark, V::Error>
            where
                V: MapAccess<'de>,
            {
                let mut id = None;
                let mut labels = None;
                let mut x = None;
                let mut y = None;
                let mut theta = None;
                let mut height = None;
                let mut width = None;

                while let Some(key) = map.next_key()? {
                    match key {
                        Field::Id => {
                            if id.is_some() {
                                return Err(de::Error::duplicate_field("id"));
                            }
                            id = Some(map.next_value()?);
                        }
                        Field::Labels => {
                            if labels.is_some() {
                                return Err(de::Error::duplicate_field("labels"));
                            }
                            labels = Some(map.next_value()?);
                        }
                        Field::X => {
                            if x.is_some() {
                                return Err(de::Error::duplicate_field("x"));
                            }
                            x = Some(map.next_value()?);
                        }
                        Field::Y => {
                            if y.is_some() {
                                return Err(de::Error::duplicate_field("y"));
                            }
                            y = Some(map.next_value()?);
                        }
                        Field::Theta => {
                            if theta.is_some() {
                                return Err(de::Error::duplicate_field("theta"));
                            }
                            theta = Some(map.next_value()?);
                        }
                        Field::Height => {
                            if height.is_some() {
                                return Err(de::Error::duplicate_field("height"));
                            }
                            height = Some(map.next_value()?);
                        }
                        Field::Width => {
                            if width.is_some() {
                                return Err(de::Error::duplicate_field("width"));
                            }
                            width = Some(map.next_value()?);
                        }
                        Field::Unknown => {}
                    }
                }
                let id = id.ok_or_else(|| de::Error::missing_field("id"))?;
                let labels = labels.unwrap_or(Vec::new());
                let x = x.ok_or_else(|| de::Error::missing_field("x"))?;
                let y = y.ok_or_else(|| de::Error::missing_field("y"))?;
                let theta = theta.unwrap_or(0.);
                let height = height.unwrap_or(1.);
                let width = width.unwrap_or(0.);
                Ok(OrientedLandmark {
                    id,
                    labels,
                    pose: Vector3::from_vec(vec![x, y, theta]),
                    height,
                    width,
                })
            }
        }

        const FIELDS: &[&str] = &["id", "labels", "x", "y", "theta", "height", "width"];
        deserializer.deserialize_struct("OrientedLandmark", FIELDS, OrientedLandmarkVisitor)
    }
}
