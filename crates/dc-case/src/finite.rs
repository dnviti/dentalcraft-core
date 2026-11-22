//! Detection of non-finite floating-point numbers in serializable data.
//!
//! JSON cannot store NaN or ±infinity: `serde_json` silently writes them as
//! `null`, which then fails to decode (or decodes as `None`). Saving checks
//! the data with [`find_non_finite`] before anything is written.

use std::fmt;

use serde::Serialize;
use serde::ser;

/// Returns the path (for example `world_frame.from_case.t[0]`) of the first
/// non-finite `f32`/`f64` in `value`, or `None` when every number is finite.
///
/// A `Serialize` implementation that fails is reported as its error text.
pub fn find_non_finite<T: Serialize + ?Sized>(value: &T) -> Option<String> {
    match value.serialize(Checker) {
        Ok(()) => None,
        Err(Found::NonFinite(mut path)) => {
            path.reverse();
            let mut out = String::new();
            for seg in path {
                if !out.is_empty() && !seg.starts_with('[') {
                    out.push('.');
                }
                out.push_str(&seg);
            }
            Some(if out.is_empty() {
                "(root)".to_owned()
            } else {
                out
            })
        }
        Err(Found::Other(msg)) => Some(msg),
    }
}

#[derive(Debug)]
enum Found {
    /// Path segments, innermost first (built while the error unwinds).
    NonFinite(Vec<String>),
    Other(String),
}

impl Found {
    fn within(self, seg: impl FnOnce() -> String) -> Self {
        match self {
            Self::NonFinite(mut path) => {
                path.push(seg());
                Self::NonFinite(path)
            }
            other => other,
        }
    }
}

impl fmt::Display for Found {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NonFinite(_) => f.write_str("non-finite number"),
            Self::Other(m) => f.write_str(m),
        }
    }
}

impl std::error::Error for Found {}

impl ser::Error for Found {
    fn custom<T: fmt::Display>(msg: T) -> Self {
        Self::Other(msg.to_string())
    }
}

/// Serializer that produces nothing and fails on the first non-finite float.
struct Checker;

/// Compound state: counts elements for the path of an error.
struct Compound {
    index: usize,
}

fn check(v: f64) -> Result<(), Found> {
    if v.is_finite() {
        Ok(())
    } else {
        Err(Found::NonFinite(Vec::new()))
    }
}

impl ser::Serializer for Checker {
    type Ok = ();
    type Error = Found;
    type SerializeSeq = Compound;
    type SerializeTuple = Compound;
    type SerializeTupleStruct = Compound;
    type SerializeTupleVariant = Compound;
    type SerializeMap = Compound;
    type SerializeStruct = Compound;
    type SerializeStructVariant = Compound;

    fn serialize_bool(self, _: bool) -> Result<(), Found> {
        Ok(())
    }
    fn serialize_i8(self, _: i8) -> Result<(), Found> {
        Ok(())
    }
    fn serialize_i16(self, _: i16) -> Result<(), Found> {
        Ok(())
    }
    fn serialize_i32(self, _: i32) -> Result<(), Found> {
        Ok(())
    }
    fn serialize_i64(self, _: i64) -> Result<(), Found> {
        Ok(())
    }
    fn serialize_i128(self, _: i128) -> Result<(), Found> {
        Ok(())
    }
    fn serialize_u8(self, _: u8) -> Result<(), Found> {
        Ok(())
    }
    fn serialize_u16(self, _: u16) -> Result<(), Found> {
        Ok(())
    }
    fn serialize_u32(self, _: u32) -> Result<(), Found> {
        Ok(())
    }
    fn serialize_u64(self, _: u64) -> Result<(), Found> {
        Ok(())
    }
    fn serialize_u128(self, _: u128) -> Result<(), Found> {
        Ok(())
    }
    fn serialize_f32(self, v: f32) -> Result<(), Found> {
        check(f64::from(v))
    }
    fn serialize_f64(self, v: f64) -> Result<(), Found> {
        check(v)
    }
    fn serialize_char(self, _: char) -> Result<(), Found> {
        Ok(())
    }
    fn serialize_str(self, _: &str) -> Result<(), Found> {
        Ok(())
    }
    fn serialize_bytes(self, _: &[u8]) -> Result<(), Found> {
        Ok(())
    }
    fn serialize_none(self) -> Result<(), Found> {
        Ok(())
    }
    fn serialize_some<T: Serialize + ?Sized>(self, value: &T) -> Result<(), Found> {
        value.serialize(self)
    }
    fn serialize_unit(self) -> Result<(), Found> {
        Ok(())
    }
    fn serialize_unit_struct(self, _: &'static str) -> Result<(), Found> {
        Ok(())
    }
    fn serialize_unit_variant(self, _: &'static str, _: u32, _: &'static str) -> Result<(), Found> {
        Ok(())
    }
    fn serialize_newtype_struct<T: Serialize + ?Sized>(
        self,
        _: &'static str,
        value: &T,
    ) -> Result<(), Found> {
        value.serialize(self)
    }
    fn serialize_newtype_variant<T: Serialize + ?Sized>(
        self,
        _: &'static str,
        _: u32,
        variant: &'static str,
        value: &T,
    ) -> Result<(), Found> {
        value
            .serialize(self)
            .map_err(|e| e.within(|| variant.to_owned()))
    }
    fn serialize_seq(self, _: Option<usize>) -> Result<Compound, Found> {
        Ok(Compound { index: 0 })
    }
    fn serialize_tuple(self, _: usize) -> Result<Compound, Found> {
        Ok(Compound { index: 0 })
    }
    fn serialize_tuple_struct(self, _: &'static str, _: usize) -> Result<Compound, Found> {
        Ok(Compound { index: 0 })
    }
    fn serialize_tuple_variant(
        self,
        _: &'static str,
        _: u32,
        _: &'static str,
        _: usize,
    ) -> Result<Compound, Found> {
        Ok(Compound { index: 0 })
    }
    fn serialize_map(self, _: Option<usize>) -> Result<Compound, Found> {
        Ok(Compound { index: 0 })
    }
    fn serialize_struct(self, _: &'static str, _: usize) -> Result<Compound, Found> {
        Ok(Compound { index: 0 })
    }
    fn serialize_struct_variant(
        self,
        _: &'static str,
        _: u32,
        _: &'static str,
        _: usize,
    ) -> Result<Compound, Found> {
        Ok(Compound { index: 0 })
    }
}

impl Compound {
    fn element<T: Serialize + ?Sized>(&mut self, value: &T) -> Result<(), Found> {
        let i = self.index;
        self.index += 1;
        value
            .serialize(Checker)
            .map_err(|e| e.within(|| format!("[{i}]")))
    }

    fn field<T: Serialize + ?Sized>(&mut self, key: &'static str, value: &T) -> Result<(), Found> {
        value
            .serialize(Checker)
            .map_err(|e| e.within(|| key.to_owned()))
    }
}

impl ser::SerializeSeq for Compound {
    type Ok = ();
    type Error = Found;
    fn serialize_element<T: Serialize + ?Sized>(&mut self, value: &T) -> Result<(), Found> {
        self.element(value)
    }
    fn end(self) -> Result<(), Found> {
        Ok(())
    }
}

impl ser::SerializeTuple for Compound {
    type Ok = ();
    type Error = Found;
    fn serialize_element<T: Serialize + ?Sized>(&mut self, value: &T) -> Result<(), Found> {
        self.element(value)
    }
    fn end(self) -> Result<(), Found> {
        Ok(())
    }
}

impl ser::SerializeTupleStruct for Compound {
    type Ok = ();
    type Error = Found;
    fn serialize_field<T: Serialize + ?Sized>(&mut self, value: &T) -> Result<(), Found> {
        self.element(value)
    }
    fn end(self) -> Result<(), Found> {
        Ok(())
    }
}

impl ser::SerializeTupleVariant for Compound {
    type Ok = ();
    type Error = Found;
    fn serialize_field<T: Serialize + ?Sized>(&mut self, value: &T) -> Result<(), Found> {
        self.element(value)
    }
    fn end(self) -> Result<(), Found> {
        Ok(())
    }
}

impl ser::SerializeMap for Compound {
    type Ok = ();
    type Error = Found;
    fn serialize_key<T: Serialize + ?Sized>(&mut self, key: &T) -> Result<(), Found> {
        key.serialize(Checker)
    }
    fn serialize_value<T: Serialize + ?Sized>(&mut self, value: &T) -> Result<(), Found> {
        self.element(value)
    }
    fn serialize_entry<K: Serialize + ?Sized, V: Serialize + ?Sized>(
        &mut self,
        key: &K,
        value: &V,
    ) -> Result<(), Found> {
        key.serialize(Checker)?;
        self.index += 1;
        // The key is only formatted when a number below it is not finite.
        value.serialize(Checker).map_err(|e| {
            e.within(|| match serde_json::to_value(key) {
                Ok(serde_json::Value::String(s)) => format!("[{s:?}]"),
                Ok(v) => format!("[{v}]"),
                Err(_) => "[?]".to_owned(),
            })
        })
    }
    fn end(self) -> Result<(), Found> {
        Ok(())
    }
}

impl ser::SerializeStruct for Compound {
    type Ok = ();
    type Error = Found;
    fn serialize_field<T: Serialize + ?Sized>(
        &mut self,
        key: &'static str,
        value: &T,
    ) -> Result<(), Found> {
        self.field(key, value)
    }
    fn end(self) -> Result<(), Found> {
        Ok(())
    }
}

impl ser::SerializeStructVariant for Compound {
    type Ok = ();
    type Error = Found;
    fn serialize_field<T: Serialize + ?Sized>(
        &mut self,
        key: &'static str,
        value: &T,
    ) -> Result<(), Found> {
        self.field(key, value)
    }
    fn end(self) -> Result<(), Found> {
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use std::collections::BTreeMap;

    use super::*;

    #[derive(Serialize)]
    struct Inner {
        a: Option<f64>,
        v: Vec<f32>,
    }

    #[derive(Serialize)]
    struct Outer {
        name: String,
        inner: Inner,
        map: BTreeMap<u8, f64>,
    }

    fn sample() -> Outer {
        Outer {
            name: "x".into(),
            inner: Inner {
                a: Some(1.0),
                v: vec![0.5, 2.0],
            },
            map: BTreeMap::from([(11, 1.0), (21, 2.0)]),
        }
    }

    #[test]
    fn finite_data_passes() {
        assert_eq!(find_non_finite(&sample()), None);
    }

    #[test]
    fn non_finite_numbers_are_found_with_their_path() {
        let mut s = sample();
        s.inner.a = Some(f64::NAN);
        assert_eq!(find_non_finite(&s).as_deref(), Some("inner.a"));
        let mut s = sample();
        s.inner.v[1] = f32::INFINITY;
        assert_eq!(find_non_finite(&s).as_deref(), Some("inner.v[1]"));
        let mut s = sample();
        s.map.insert(21, f64::NEG_INFINITY);
        assert_eq!(find_non_finite(&s).as_deref(), Some("map[21]"));
        assert_eq!(find_non_finite(&f64::NAN).as_deref(), Some("(root)"));
    }
}
