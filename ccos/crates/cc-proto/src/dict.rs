//! Message formats, the fixed Centauri Carbon id table, and message
//! encode/decode.

use std::collections::HashMap;

use crate::{vlq, Error, MESSAGE_PAYLOAD_MAX};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ParamType {
    U32,
    I32,
    U16,
    I16,
    Byte,
    /// `%s`, `%*s`, `%.*s`: length byte followed by bytes.
    Buffer,
}

impl ParamType {
    fn from_spec(s: &str) -> Option<Self> {
        Some(match s {
            "%u" => Self::U32,
            "%i" => Self::I32,
            "%hu" => Self::U16,
            "%hi" => Self::I16,
            "%c" => Self::Byte,
            "%s" | "%*s" | "%.*s" => Self::Buffer,
            _ => return None,
        })
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Arg {
    Int(i64),
    Bytes(Vec<u8>),
}

impl Arg {
    pub fn as_int(&self) -> Option<i64> {
        match self {
            Arg::Int(v) => Some(*v),
            Arg::Bytes(_) => None,
        }
    }
    pub fn as_bytes(&self) -> Option<&[u8]> {
        match self {
            Arg::Bytes(b) => Some(b),
            Arg::Int(_) => None,
        }
    }
}

impl std::fmt::Display for Arg {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Arg::Int(v) => write!(f, "{v}"),
            Arg::Bytes(b) => match std::str::from_utf8(b) {
                Ok(s) if s.chars().all(|c| !c.is_control()) => write!(f, "{s:?}"),
                _ => {
                    write!(f, "0x")?;
                    b.iter().try_for_each(|x| write!(f, "{x:02x}"))
                }
            },
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MessageFormat {
    pub id: u32,
    pub name: String,
    pub params: Vec<(String, ParamType)>,
    pub format: String,
}

impl MessageFormat {
    pub fn parse(id: u32, format: &str) -> Result<Self, Error> {
        let mut parts = format.split_whitespace();
        let name = parts.next().ok_or_else(|| Error::BadFormat(format.into()))?.to_string();
        let params = parts
            .map(|p| {
                let (k, spec) = p.split_once('=').ok_or_else(|| Error::BadFormat(format.into()))?;
                let t = ParamType::from_spec(spec).ok_or_else(|| Error::BadFormat(format.into()))?;
                Ok((k.to_string(), t))
            })
            .collect::<Result<_, Error>>()?;
        Ok(Self { id, name, params, format: format.trim().to_string() })
    }
}

/// A decoded (or to-be-encoded) message.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Message {
    pub name: String,
    pub args: Vec<(String, Arg)>,
}

impl Message {
    pub fn get(&self, key: &str) -> Option<&Arg> {
        self.args.iter().find(|(k, _)| k == key).map(|(_, v)| v)
    }
    pub fn int(&self, key: &str) -> Option<i64> {
        self.get(key).and_then(Arg::as_int)
    }
}

impl std::fmt::Display for Message {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.name)?;
        for (k, v) in &self.args {
            write!(f, " {k}={v}")?;
        }
        Ok(())
    }
}

#[derive(Debug, Clone, Default)]
pub struct Dictionary {
    by_id: HashMap<u32, MessageFormat>,
    by_name: HashMap<String, u32>,
    pub static_strings: HashMap<u32, String>,
}

const CENTAURI_TABLE: &str = include_str!("../data/command_table.json");
const CENTAURI_STRINGS: &str = include_str!("../data/static_strings.json");

impl Dictionary {
    /// The fixed id table compiled into Elegoo's host and all three MCU
    /// firmwares.
    pub fn centauri() -> Self {
        let table: HashMap<String, String> = serde_json::from_str(CENTAURI_TABLE).expect("embedded table");
        let strings: HashMap<String, String> = serde_json::from_str(CENTAURI_STRINGS).expect("embedded strings");
        let mut d = Self::from_formats(table.iter().map(|(k, v)| (k.parse().unwrap(), v.as_str())))
            .expect("embedded table parses");
        d.static_strings = strings.into_iter().map(|(k, v)| (k.parse().unwrap(), v)).collect();
        d
    }

    pub fn from_formats<'a>(formats: impl IntoIterator<Item = (u32, &'a str)>) -> Result<Self, Error> {
        let mut d = Self::default();
        for (id, fmt) in formats {
            let mf = MessageFormat::parse(id, fmt)?;
            d.by_name.insert(mf.name.clone(), id);
            d.by_id.insert(id, mf);
        }
        Ok(d)
    }

    /// Build from an upstream-style identify blob (zlib-compressed JSON).
    /// Stock Centauri firmware ships stale blobs; this is for custom MCU builds.
    pub fn from_identify(zlib_data: &[u8]) -> Result<Self, Error> {
        let raw = miniz_oxide::inflate::decompress_to_vec_zlib(zlib_data)
            .map_err(|e| Error::BadFormat(format!("inflate: {e:?}")))?;
        let v: serde_json::Value = serde_json::from_slice(&raw).map_err(|e| Error::BadFormat(e.to_string()))?;
        let mut formats = Vec::new();
        for key in ["commands", "responses", "output"] {
            if let Some(obj) = v.get(key).and_then(|o| o.as_object()) {
                for (fmt, id) in obj {
                    if let Some(id) = id.as_u64() {
                        formats.push((id as u32, fmt.clone()));
                    }
                }
            }
        }
        Self::from_formats(formats.iter().map(|(i, f)| (*i, f.as_str())))
    }

    pub fn len(&self) -> usize {
        self.by_id.len()
    }
    pub fn is_empty(&self) -> bool {
        self.by_id.is_empty()
    }
    pub fn by_name(&self, name: &str) -> Option<&MessageFormat> {
        self.by_name.get(name).and_then(|id| self.by_id.get(id))
    }
    pub fn by_id(&self, id: u32) -> Option<&MessageFormat> {
        self.by_id.get(&id)
    }
    pub fn formats(&self) -> impl Iterator<Item = &MessageFormat> {
        self.by_id.values()
    }

    /// Encode a message given typed arguments (order-independent, by name).
    pub fn encode(&self, name: &str, args: &[(&str, Arg)]) -> Result<Vec<u8>, Error> {
        let mf = self.by_name(name).ok_or_else(|| Error::UnknownMessage(name.into()))?;
        let mut out = Vec::new();
        vlq::encode(&mut out, mf.id);
        for (pname, ptype) in &mf.params {
            let arg = args
                .iter()
                .find(|(k, _)| k == pname)
                .map(|(_, v)| v)
                .ok_or_else(|| Error::MissingParam { msg: name.into(), param: pname.clone() })?;
            match (ptype, arg) {
                (ParamType::Buffer, Arg::Bytes(b)) => {
                    if b.len() > MESSAGE_PAYLOAD_MAX {
                        return Err(Error::TooLong(b.len()));
                    }
                    out.push(b.len() as u8);
                    out.extend_from_slice(b);
                }
                (ParamType::Buffer, Arg::Int(_)) | (_, Arg::Bytes(_)) => {
                    return Err(Error::BadValue { param: pname.clone(), value: arg.to_string() })
                }
                (_, Arg::Int(v)) => vlq::encode(&mut out, *v as u32),
            }
        }
        Ok(out)
    }

    /// Encode a Klipper-style text command, e.g. `"set_digital_out pin=3 value=1"`.
    /// Buffer parameters take a `0x`-prefixed hex string or literal text.
    pub fn encode_str(&self, cmd: &str) -> Result<Vec<u8>, Error> {
        let mut parts = cmd.split_whitespace();
        let name = parts.next().ok_or_else(|| Error::UnknownMessage(String::new()))?;
        let mf = self.by_name(name).ok_or_else(|| Error::UnknownMessage(name.into()))?;
        let mut args = Vec::new();
        for p in parts {
            let (k, v) = p.split_once('=').ok_or_else(|| Error::BadValue { param: p.into(), value: p.into() })?;
            let ptype = mf.params.iter().find(|(n, _)| n == k).map(|(_, t)| *t);
            let arg = match ptype {
                Some(ParamType::Buffer) => Arg::Bytes(parse_bytes(v)),
                _ => Arg::Int(parse_int(v).ok_or_else(|| Error::BadValue { param: k.into(), value: v.into() })?),
            };
            args.push((k, arg));
        }
        self.encode(name, &args)
    }

    /// Decode every message in a frame payload.
    pub fn decode_payload(&self, mut payload: &[u8]) -> Result<Vec<Message>, Error> {
        let mut msgs = Vec::new();
        while !payload.is_empty() {
            let (id, n) = vlq::decode(payload).ok_or(Error::Truncated)?;
            payload = &payload[n..];
            let mf = self.by_id(id).ok_or(Error::UnknownId(id))?;
            let mut args = Vec::with_capacity(mf.params.len());
            for (pname, ptype) in &mf.params {
                let arg = match ptype {
                    ParamType::Buffer => {
                        let len = *payload.first().ok_or(Error::Truncated)? as usize;
                        let b = payload.get(1..1 + len).ok_or(Error::Truncated)?.to_vec();
                        payload = &payload[1 + len..];
                        Arg::Bytes(b)
                    }
                    t => {
                        let (v, n) = vlq::decode(payload).ok_or(Error::Truncated)?;
                        payload = &payload[n..];
                        Arg::Int(match t {
                            ParamType::U32 => v as i64,
                            ParamType::I32 => v as i32 as i64,
                            ParamType::U16 => v as u16 as i64,
                            ParamType::I16 => v as i16 as i64,
                            ParamType::Byte => v as u8 as i64,
                            ParamType::Buffer => unreachable!(),
                        })
                    }
                };
                args.push((pname.clone(), arg));
            }
            msgs.push(Message { name: mf.name.clone(), args });
        }
        Ok(msgs)
    }
}

fn parse_int(v: &str) -> Option<i64> {
    if let Some(h) = v.strip_prefix("0x") {
        i64::from_str_radix(h, 16).ok()
    } else {
        v.parse().ok()
    }
}

fn parse_bytes(v: &str) -> Vec<u8> {
    if let Some(h) = v.strip_prefix("0x") {
        if h.len() % 2 == 0 {
            if let Some(b) = (0..h.len()).step_by(2).map(|i| u8::from_str_radix(&h[i..i + 2], 16).ok()).collect() {
                return b;
            }
        }
    }
    v.as_bytes().to_vec()
}

/// Map a pin name like `PE8` to the MCU pin number used on the wire.
///
/// `pins_per_bank` is 32 for the R528 DSP and 16 for the STM32 boards
/// (see `mcu.cpp`). `ADC_TEMPERATURE` is 254 on all MCUs.
pub fn pin_number(name: &str, pins_per_bank: u32) -> Option<u32> {
    let name = name.trim().trim_start_matches(['!', '^', '~']);
    if name == "ADC_TEMPERATURE" {
        return Some(254);
    }
    let rest = name.strip_prefix('P')?;
    let mut chars = rest.chars();
    let bank = chars.next()?;
    if !bank.is_ascii_uppercase() {
        return None;
    }
    let idx: u32 = chars.as_str().parse().ok()?;
    if idx >= pins_per_bank {
        return None;
    }
    Some((bank as u32 - 'A' as u32) * pins_per_bank + idx)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn centauri_table_loads() {
        let d = Dictionary::centauri();
        assert_eq!(d.len(), 115);
        assert_eq!(d.by_name("identify").unwrap().id, 1);
        assert_eq!(d.by_name("identify_response").unwrap().id, 0);
        assert!(d.by_name("clear_shutdown").unwrap().params.is_empty());
        assert_eq!(d.static_strings.get(&3).map(String::as_str), Some("Timer too close"));
    }

    #[test]
    fn encode_identify_matches_c_frame_payload() {
        let d = Dictionary::centauri();
        assert_eq!(d.encode_str("identify offset=0 count=40").unwrap(), vec![0x01, 0x00, 0x28]);
    }

    #[test]
    fn decode_identify_response() {
        let d = Dictionary::centauri();
        let payload = [0x00, 0x00, 0x03, b'a', b'b', b'c'];
        let msgs = d.decode_payload(&payload).unwrap();
        assert_eq!(msgs.len(), 1);
        assert_eq!(msgs[0].to_string(), "identify_response offset=0 data=\"abc\"");
    }

    #[test]
    fn roundtrip_every_format() {
        let d = Dictionary::centauri();
        for mf in d.formats() {
            let args: Vec<(&str, Arg)> = mf
                .params
                .iter()
                .enumerate()
                .map(|(i, (n, t))| {
                    let a = match t {
                        ParamType::Buffer => Arg::Bytes(vec![i as u8; 3]),
                        ParamType::Byte => Arg::Int(200),
                        ParamType::U16 => Arg::Int(60000),
                        ParamType::I16 => Arg::Int(-30000),
                        ParamType::I32 => Arg::Int(-123456),
                        ParamType::U32 => Arg::Int(4_000_000_000),
                    };
                    (n.as_str(), a)
                })
                .collect();
            let enc = d.encode(&mf.name, &args).unwrap();
            let dec = d.decode_payload(&enc).unwrap();
            assert_eq!(dec.len(), 1, "{}", mf.format);
            for ((n, a), (dn, da)) in args.iter().zip(&dec[0].args) {
                assert_eq!((n, a), (&dn.as_str(), da), "{}", mf.format);
            }
        }
    }

    #[test]
    fn multiple_messages_in_one_payload() {
        let d = Dictionary::centauri();
        let mut p = d.encode_str("get_uptime").unwrap();
        p.extend(d.encode_str("identify offset=40 count=40").unwrap());
        let m = d.decode_payload(&p).unwrap();
        assert_eq!(m.iter().map(|m| m.name.as_str()).collect::<Vec<_>>(), ["get_uptime", "identify"]);
        assert_eq!(m[1].int("offset"), Some(40));
    }

    #[test]
    fn pins() {
        // printer.cfg: stepper_x step_pin PE8 on the DSP (32 pins/bank)
        assert_eq!(pin_number("PE8", 32), Some(4 * 32 + 8));
        assert_eq!(pin_number("!PE7", 32), Some(4 * 32 + 7));
        // toolhead heater stm32:PB6 (16 pins/bank)
        assert_eq!(pin_number("PB6", 16), Some(22));
        assert_eq!(pin_number("PA16", 16), None);
        assert_eq!(pin_number("ADC_TEMPERATURE", 16), Some(254));
    }

    #[test]
    fn stale_identify_blob_inflates() {
        // Build a tiny identify blob and make sure the upstream path works.
        let json = br#"{"commands":{"identify offset=%u count=%c":1},"responses":{"identify_response offset=%u data=%.*s":0}}"#;
        let z = miniz_oxide::deflate::compress_to_vec_zlib(json, 6);
        let d = Dictionary::from_identify(&z).unwrap();
        assert_eq!(d.len(), 2);
    }
}
