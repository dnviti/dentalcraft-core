//! FDI (ISO 3950) tooth numbering.

use std::fmt;

use serde::{Deserialize, Serialize};

use crate::CaseError;

/// FDI tooth number: quadrant digit followed by position digit.
///
/// Quadrants 1–4 are permanent teeth (positions 1–8), 5–8 deciduous teeth
/// (positions 1–5). Quadrants 1/5 and 2/6 are upper, 3/7 and 4/8 lower;
/// 1/5 and 4/8 are on the patient's right.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(try_from = "u8", into = "u8")]
pub struct Fdi(u8);

const fn valid(n: u8) -> bool {
    let (q, p) = (n / 10, n % 10);
    match q {
        1..=4 => p >= 1 && p <= 8,
        5..=8 => p >= 1 && p <= 5,
        _ => false,
    }
}

const fn permanent() -> [Fdi; 32] {
    let mut out = [Fdi(11); 32];
    let mut i = 0;
    while i < 32 {
        out[i] = Fdi(((i / 8) as u8 + 1) * 10 + (i % 8) as u8 + 1);
        i += 1;
    }
    out
}

impl Fdi {
    /// The 32 permanent teeth, quadrant by quadrant (11–18, 21–28, 31–38, 41–48).
    pub const PERMANENT: [Fdi; 32] = permanent();

    /// Validated FDI number.
    pub fn new(n: u8) -> Result<Self, CaseError> {
        if valid(n) {
            Ok(Self(n))
        } else {
            Err(CaseError::InvalidFdi(n))
        }
    }

    /// The numeric value (e.g. `11`).
    pub fn get(self) -> u8 {
        self.0
    }

    /// Quadrant digit (1–8).
    pub fn quadrant(self) -> u8 {
        self.0 / 10
    }

    /// Position digit from the midline (1 = central incisor).
    pub fn position(self) -> u8 {
        self.0 % 10
    }

    /// `true` for permanent teeth (quadrants 1–4).
    pub fn is_permanent(self) -> bool {
        self.quadrant() <= 4
    }

    /// `true` for upper teeth (quadrants 1, 2, 5, 6).
    pub fn is_upper(self) -> bool {
        matches!(self.quadrant(), 1 | 2 | 5 | 6)
    }

    /// `true` for teeth on the patient's right (quadrants 1, 4, 5, 8).
    pub fn is_right(self) -> bool {
        matches!(self.quadrant(), 1 | 4 | 5 | 8)
    }

    /// Same position on the other side of the midline (11 ↔ 21, 36 ↔ 46).
    pub fn mirror(self) -> Self {
        let q = match self.quadrant() {
            1 => 2,
            2 => 1,
            3 => 4,
            4 => 3,
            5 => 6,
            6 => 5,
            7 => 8,
            _ => 7,
        };
        Self(q * 10 + self.position())
    }

    /// Permanent teeth of one arch from the patient's right distal to the
    /// left distal (upper: 18…11, 21…28; lower: 48…41, 31…38).
    pub fn permanent_arch(upper: bool) -> impl Iterator<Item = Fdi> {
        let (right, left) = if upper { (1u8, 2u8) } else { (4, 3) };
        (1..=8u8)
            .rev()
            .map(move |p| Fdi(right * 10 + p))
            .chain((1..=8u8).map(move |p| Fdi(left * 10 + p)))
    }
}

impl TryFrom<u8> for Fdi {
    type Error = CaseError;

    fn try_from(n: u8) -> Result<Self, CaseError> {
        Self::new(n)
    }
}

impl From<Fdi> for u8 {
    fn from(f: Fdi) -> u8 {
        f.0
    }
}

impl fmt::Display for Fdi {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn validation() {
        for n in [11, 18, 28, 31, 48, 51, 55, 85] {
            assert!(Fdi::new(n).is_ok(), "{n}");
        }
        for n in [0, 10, 19, 56, 90, 9, 99] {
            assert!(Fdi::new(n).is_err(), "{n}");
        }
    }

    #[test]
    fn helpers() {
        let t = Fdi::new(36).unwrap();
        assert_eq!((t.quadrant(), t.position()), (3, 6));
        assert!(!t.is_upper() && !t.is_right() && t.is_permanent());
        assert_eq!(t.mirror().get(), 46);
        assert!(Fdi::new(55).unwrap().is_upper());
        assert!(!Fdi::new(55).unwrap().is_permanent());
    }

    #[test]
    fn permanent_dentition() {
        assert_eq!(Fdi::PERMANENT.len(), 32);
        assert_eq!(Fdi::PERMANENT[0].get(), 11);
        assert_eq!(Fdi::PERMANENT[31].get(), 48);
        assert!(Fdi::PERMANENT.iter().all(|f| Fdi::new(f.get()).is_ok()));
        let upper: Vec<u8> = Fdi::permanent_arch(true).map(Fdi::get).collect();
        assert_eq!(upper.first(), Some(&18));
        assert_eq!(upper[7..9], [11, 21]);
        assert_eq!(upper.last(), Some(&28));
        let lower: Vec<u8> = Fdi::permanent_arch(false).map(Fdi::get).collect();
        assert_eq!((lower[0], lower[15]), (48, 38));
    }

    #[test]
    fn serde_validates() {
        assert_eq!(serde_json::to_string(&Fdi::new(21).unwrap()).unwrap(), "21");
        assert!(serde_json::from_str::<Fdi>("19").is_err());
        assert_eq!(serde_json::from_str::<Fdi>("47").unwrap().get(), 47);
    }
}

#[cfg(kani)]
mod kani_proofs {
    use super::*;

    /// Valid FDI numbers written out, independently of `valid`.
    fn in_spec(n: u8) -> bool {
        matches!(
            n,
            11..=18
                | 21..=28
                | 31..=38
                | 41..=48
                | 51..=55
                | 61..=65
                | 71..=75
                | 81..=85
        )
    }

    fn any_fdi() -> Fdi {
        let n: u8 = kani::any();
        let f = Fdi::new(n);
        kani::assume(f.is_ok());
        f.unwrap()
    }

    // `Fdi::new` accepts exactly the ISO 3950 numbers (permanent 11–48,
    // deciduous 51–85), and an accepted number keeps its value and digits.
    #[kani::proof]
    fn new_accepts_exactly_iso_numbers() {
        let n: u8 = kani::any();
        let f = Fdi::new(n);
        assert_eq!(f.is_ok(), in_spec(n));
        if let Ok(f) = f {
            assert_eq!(f.get(), n);
            assert_eq!(f.quadrant() * 10 + f.position(), n);
            assert!((1..=8).contains(&f.quadrant()));
            assert!(f.position() >= 1);
            assert_eq!(Fdi::try_from(n).ok(), Some(f));
        }
    }

    // The mirror of a valid tooth is a different valid tooth, and mirroring
    // twice gives the original tooth back.
    #[kani::proof]
    fn mirror_is_valid_involution() {
        let f = any_fdi();
        let m = f.mirror();
        assert!(in_spec(m.get()));
        assert_ne!(m, f);
        assert_eq!(m.mirror(), f);
    }

    // Mirroring keeps the arch, the dentition and the position and flips the
    // side of the midline.
    #[kani::proof]
    fn mirror_keeps_arch_and_flips_side() {
        let f = any_fdi();
        let m = f.mirror();
        assert_eq!(m.is_upper(), f.is_upper());
        assert_eq!(m.is_permanent(), f.is_permanent());
        assert_eq!(m.position(), f.position());
        assert_ne!(m.is_right(), f.is_right());
    }

    // `permanent_arch(upper)` lists 16 distinct valid permanent teeth, all of
    // that arch. (Distinctness via a bit set: comparing every pair takes
    // minutes.)
    #[kani::proof]
    #[kani::unwind(18)]
    fn permanent_arch_is_the_16_teeth_of_the_arch() {
        let upper: bool = kani::any();
        let mut seen = 0u128;
        let mut count = 0;
        for f in Fdi::permanent_arch(upper) {
            assert!(in_spec(f.get()));
            assert!(f.is_permanent());
            assert_eq!(f.is_upper(), upper);
            assert_eq!(seen & (1 << f.get()), 0);
            seen |= 1 << f.get();
            count += 1;
        }
        assert_eq!(count, 16);
    }
}
