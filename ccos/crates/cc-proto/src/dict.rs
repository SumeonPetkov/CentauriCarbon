//! Message formats, the fixed Centauri Carbon id table, and allocation-free
//! message encode/decode.

use crate::{vlq, Error, MESSAGE_PAYLOAD_MAX};

/// Maximum parameters in one message (`config_fancheck` has 12).
pub const MAX_ARGS: usize = 12;

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
    pub fn from_spec(s: &str) -> Option<Self> {
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

#[derive(Debug, PartialEq, Eq)]
pub struct MessageFormat {
    pub id: u16,
    pub name: &'static str,
    pub params: &'static [(&'static str, ParamType)],
    pub format: &'static str,
}

include!(concat!(env!("OUT_DIR"), "/centauri_table.rs"));

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Arg<'a> {
    Int(i64),
    Bytes(&'a [u8]),
}

impl<'a> Arg<'a> {
    pub fn as_int(&self) -> Option<i64> {
        match self {
            Arg::Int(v) => Some(*v),
            Arg::Bytes(_) => None,
        }
    }
    pub fn as_bytes(&self) -> Option<&'a [u8]> {
        match self {
            Arg::Bytes(b) => Some(b),
            Arg::Int(_) => None,
        }
    }
}

impl core::fmt::Display for Arg<'_> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Arg::Int(v) => write!(f, "{v}"),
            Arg::Bytes(b) => match core::str::from_utf8(b) {
                Ok(s) if s.chars().all(|c| !c.is_control()) => write!(f, "{s:?}"),
                _ => {
                    write!(f, "0x")?;
                    b.iter().try_for_each(|x| write!(f, "{x:02x}"))
                }
            },
        }
    }
}

/// Fixed-capacity byte buffer for one frame's payload (no heap).
#[derive(Clone, Copy)]
pub struct MsgBuf {
    buf: [u8; MESSAGE_PAYLOAD_MAX],
    len: u8,
}

impl Default for MsgBuf {
    fn default() -> Self {
        Self { buf: [0; MESSAGE_PAYLOAD_MAX], len: 0 }
    }
}

impl MsgBuf {
    pub fn new() -> Self {
        Self::default()
    }
    pub fn from_slice(s: &[u8]) -> Result<Self, Error> {
        let mut b = Self::new();
        b.extend(s)?;
        Ok(b)
    }
    pub fn as_slice(&self) -> &[u8] {
        &self.buf[..self.len as usize]
    }
    pub fn len(&self) -> usize {
        self.len as usize
    }
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }
    pub fn clear(&mut self) {
        self.len = 0;
    }
    pub fn remaining(&self) -> usize {
        MESSAGE_PAYLOAD_MAX - self.len as usize
    }
    pub fn push(&mut self, b: u8) -> Result<(), Error> {
        if self.remaining() == 0 {
            return Err(Error::TooLong(MESSAGE_PAYLOAD_MAX + 1));
        }
        self.buf[self.len as usize] = b;
        self.len += 1;
        Ok(())
    }
    pub fn extend(&mut self, s: &[u8]) -> Result<(), Error> {
        if s.len() > self.remaining() {
            return Err(Error::TooLong(self.len as usize + s.len()));
        }
        self.buf[self.len as usize..self.len as usize + s.len()].copy_from_slice(s);
        self.len += s.len() as u8;
        Ok(())
    }
    fn vlq(&mut self, v: u32) -> Result<(), Error> {
        let mut tmp = [0u8; 5];
        let n = vlq::encode_into(&mut tmp, v);
        self.extend(&tmp[..n])
    }
}

impl core::fmt::Debug for MsgBuf {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "{:02x?}", self.as_slice())
    }
}

/// A decoded message; borrows buffers from the payload it came from.
#[derive(Clone, Copy)]
pub struct Message<'a> {
    pub fmt: &'static MessageFormat,
    args: [Arg<'a>; MAX_ARGS],
    nargs: u8,
    /// The encoded bytes of this one message (id + parameters).
    pub raw: &'a [u8],
}

impl<'a> Message<'a> {
    pub fn name(&self) -> &'static str {
        self.fmt.name
    }
    pub fn args(&self) -> impl Iterator<Item = (&'static str, Arg<'a>)> + '_ {
        self.fmt.params.iter().zip(&self.args[..self.nargs as usize]).map(|((n, _), a)| (*n, *a))
    }
    pub fn get(&self, key: &str) -> Option<Arg<'a>> {
        self.args().find(|(k, _)| *k == key).map(|(_, v)| v)
    }
    pub fn int(&self, key: &str) -> Option<i64> {
        self.get(key).and_then(|a| a.as_int())
    }
    pub fn bytes(&self, key: &str) -> Option<&'a [u8]> {
        self.get(key).and_then(|a| a.as_bytes())
    }
    /// Copy into an owned, fixed-size form (for queues).
    pub fn to_owned(&self) -> OwnedMessage {
        OwnedMessage { fmt: self.fmt, raw: MsgBuf::from_slice(self.raw).expect("message fits a frame") }
    }
}

impl core::fmt::Display for Message<'_> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "{}", self.fmt.name)?;
        for (k, v) in self.args() {
            write!(f, " {k}={v}")?;
        }
        Ok(())
    }
}

/// A message stored by value (≤ 59 bytes), decodable again on demand.
#[derive(Clone, Copy, Debug)]
pub struct OwnedMessage {
    pub fmt: &'static MessageFormat,
    pub raw: MsgBuf,
}

impl OwnedMessage {
    pub fn name(&self) -> &'static str {
        self.fmt.name
    }
    pub fn message(&self) -> Message<'_> {
        decode_one(self.fmt, self.raw.as_slice(), vlq::decode(self.raw.as_slice()).map_or(0, |(_, n)| n))
            .map(|(m, _)| m)
            .expect("re-decoding a message that decoded before")
    }
    pub fn int(&self, key: &str) -> Option<i64> {
        self.message().int(key)
    }
}

/// Decode the parameters of `fmt` starting at `raw[start..]`.
fn decode_one<'a>(fmt: &'static MessageFormat, raw: &'a [u8], start: usize) -> Result<(Message<'a>, usize), Error> {
    let mut args = [Arg::Int(0); MAX_ARGS];
    let mut p = start;
    for (i, (_, t)) in fmt.params.iter().enumerate() {
        args[i] = match t {
            ParamType::Buffer => {
                let len = *raw.get(p).ok_or(Error::Truncated)? as usize;
                let b = raw.get(p + 1..p + 1 + len).ok_or(Error::Truncated)?;
                p += 1 + len;
                Arg::Bytes(b)
            }
            t => {
                let (v, n) = vlq::decode(&raw[p.min(raw.len())..]).ok_or(Error::Truncated)?;
                p += n;
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
    }
    Ok((Message { fmt, args, nargs: fmt.params.len() as u8, raw: &raw[..p] }, p))
}

/// Iterator over the messages in one frame payload.
pub struct Messages<'a, 'd> {
    dict: &'d Dictionary,
    rest: &'a [u8],
}

impl<'a> Iterator for Messages<'a, '_> {
    type Item = Result<Message<'a>, Error>;
    fn next(&mut self) -> Option<Self::Item> {
        if self.rest.is_empty() {
            return None;
        }
        let r = (|| {
            let (id, n) = vlq::decode(self.rest).ok_or(Error::Truncated)?;
            let fmt = self.dict.by_id(id).ok_or(Error::UnknownId(id))?;
            let (m, used) = decode_one(fmt, self.rest, n)?;
            self.rest = &self.rest[used..];
            Ok(m)
        })();
        if r.is_err() {
            self.rest = &[];
        }
        Some(r)
    }
}

/// A set of message formats. The stock printer's is [`Dictionary::centauri`]
/// (static, zero cost); the `identify` feature can build one from an
/// upstream Klipper identify blob.
#[derive(Clone, Copy)]
pub struct Dictionary {
    formats: &'static [MessageFormat],
    by_name: &'static [(&'static str, u16)],
    pub static_strings: &'static [(u16, &'static str)],
}

impl Dictionary {
    /// The fixed id table compiled into Elegoo's host and all three MCU
    /// firmwares.
    pub const fn centauri() -> Self {
        Self { formats: &CENTAURI_FORMATS, by_name: &CENTAURI_BY_NAME, static_strings: &CENTAURI_STATIC_STRINGS }
    }

    pub fn len(&self) -> usize {
        self.formats.len()
    }
    pub fn is_empty(&self) -> bool {
        self.formats.is_empty()
    }
    pub fn formats(&self) -> &'static [MessageFormat] {
        self.formats
    }

    pub fn by_id(&self, id: u32) -> Option<&'static MessageFormat> {
        // Dense tables (the stock one) hit on the first probe.
        match self.formats.get(id as usize) {
            Some(f) if f.id as u32 == id => Some(f),
            _ => self.formats.binary_search_by_key(&id, |f| f.id as u32).ok().map(|i| &self.formats[i]),
        }
    }

    pub fn by_name(&self, name: &str) -> Option<&'static MessageFormat> {
        self.by_name.binary_search_by_key(&name, |(n, _)| n).ok().map(|i| &self.formats[self.by_name[i].1 as usize])
    }

    pub fn static_string(&self, id: u16) -> Option<&'static str> {
        self.static_strings.binary_search_by_key(&id, |(k, _)| *k).ok().map(|i| self.static_strings[i].1)
    }

    /// Append one message to `out`; arguments are matched by name.
    pub fn encode(&self, name: &str, args: &[(&str, Arg<'_>)], out: &mut MsgBuf) -> Result<(), Error> {
        let fmt = self.by_name(name).ok_or(Error::UnknownMessage)?;
        out.vlq(fmt.id as u32)?;
        for (pname, ptype) in fmt.params {
            let arg = args.iter().find(|(k, _)| k == pname).map(|(_, v)| *v).ok_or(Error::MissingParam(pname))?;
            match (ptype, arg) {
                (ParamType::Buffer, Arg::Bytes(b)) => {
                    out.push(u8::try_from(b.len()).map_err(|_| Error::TooLong(b.len()))?)?;
                    out.extend(b)?;
                }
                (_, Arg::Int(v)) if *ptype != ParamType::Buffer => out.vlq(v as u32)?,
                _ => return Err(Error::BadValue(pname)),
            }
        }
        Ok(())
    }

    /// Append a Klipper-style text command, e.g. `"set_digital_out pin=3 value=1"`.
    /// Buffer parameters take `0x`-prefixed hex or literal text.
    pub fn encode_str(&self, cmd: &str, out: &mut MsgBuf) -> Result<(), Error> {
        let mut parts = cmd.split_whitespace();
        let name = parts.next().ok_or(Error::UnknownMessage)?;
        let fmt = self.by_name(name).ok_or(Error::UnknownMessage)?;
        let mut words: [(&str, &str); MAX_ARGS] = [("", ""); MAX_ARGS];
        let mut n = 0;
        for p in parts {
            let (k, v) = p.split_once('=').ok_or(Error::BadSyntax)?;
            if n == MAX_ARGS {
                return Err(Error::BadSyntax);
            }
            words[n] = (k, v);
            n += 1;
        }
        out.vlq(fmt.id as u32)?;
        for (pname, ptype) in fmt.params {
            let v = words[..n].iter().find(|(k, _)| k == pname).map(|(_, v)| *v).ok_or(Error::MissingParam(pname))?;
            if *ptype == ParamType::Buffer {
                match v.strip_prefix("0x").filter(|h| h.len() % 2 == 0 && h.bytes().all(|c| c.is_ascii_hexdigit())) {
                    Some(h) => {
                        out.push((h.len() / 2) as u8)?;
                        for i in (0..h.len()).step_by(2) {
                            out.push(u8::from_str_radix(&h[i..i + 2], 16).unwrap())?;
                        }
                    }
                    None => {
                        out.push(u8::try_from(v.len()).map_err(|_| Error::TooLong(v.len()))?)?;
                        out.extend(v.as_bytes())?;
                    }
                }
            } else {
                let iv = match v.strip_prefix("0x") {
                    Some(h) => i64::from_str_radix(h, 16),
                    None => v.parse::<i64>(),
                }
                .map_err(|_| Error::BadValue(pname))?;
                out.vlq(iv as u32)?;
            }
        }
        Ok(())
    }

    /// Iterate over every message in a frame payload. No allocation.
    pub fn decode<'a, 'd>(&'d self, payload: &'a [u8]) -> Messages<'a, 'd> {
        Messages { dict: self, rest: payload }
    }

    /// Build a dictionary from an upstream-style identify blob
    /// (zlib-compressed JSON). Stock Centauri firmware ships stale blobs;
    /// this is for custom MCU builds. Leaks the (small) table on purpose.
    #[cfg(feature = "identify")]
    pub fn from_identify(zlib_data: &[u8]) -> Result<Self, Error> {
        let raw = miniz_oxide::inflate::decompress_to_vec_zlib(zlib_data).map_err(|_| Error::BadSyntax)?;
        let v: serde_json::Value = serde_json::from_slice(&raw).map_err(|_| Error::BadSyntax)?;
        let mut formats: Vec<MessageFormat> = Vec::new();
        for key in ["commands", "responses", "output"] {
            if let Some(obj) = v.get(key).and_then(|o| o.as_object()) {
                for (fmt, id) in obj {
                    let Some(id) = id.as_u64() else { continue };
                    let fmt: &'static str = Box::leak(fmt.clone().into_boxed_str());
                    let mut parts = fmt.split_whitespace();
                    let name = parts.next().ok_or(Error::BadSyntax)?;
                    let params: Vec<(&'static str, ParamType)> = parts
                        .map(|p| {
                            let (k, s) = p.split_once('=').ok_or(Error::BadSyntax)?;
                            Ok((k, ParamType::from_spec(s).ok_or(Error::BadSyntax)?))
                        })
                        .collect::<Result<_, Error>>()?;
                    if params.len() > MAX_ARGS {
                        return Err(Error::BadSyntax);
                    }
                    formats.push(MessageFormat { id: id as u16, name, params: Box::leak(params.into_boxed_slice()), format: fmt });
                }
            }
        }
        formats.sort_by_key(|f| f.id);
        let formats: &'static [MessageFormat] = Box::leak(formats.into_boxed_slice());
        let mut by_name: Vec<(&'static str, u16)> = formats.iter().enumerate().map(|(i, f)| (f.name, i as u16)).collect();
        by_name.sort();
        Ok(Self { formats, by_name: Box::leak(by_name.into_boxed_slice()), static_strings: &[] })
    }
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

    const D: Dictionary = Dictionary::centauri();

    fn enc(cmd: &str) -> MsgBuf {
        let mut b = MsgBuf::new();
        D.encode_str(cmd, &mut b).unwrap();
        b
    }

    #[test]
    fn centauri_table_loads() {
        assert_eq!(D.len(), 115);
        for (i, f) in D.formats().iter().enumerate() {
            assert_eq!(f.id as usize, i, "table is dense");
            assert_eq!(D.by_name(f.name).unwrap().id, f.id);
        }
        assert_eq!(D.by_name("identify").unwrap().id, 1);
        assert_eq!(D.by_name("identify_response").unwrap().id, 0);
        assert!(D.by_name("clear_shutdown").unwrap().params.is_empty());
        assert_eq!(D.static_string(3), Some("Timer too close"));
        assert!(D.by_name("nope").is_none());
    }

    #[test]
    fn encode_identify_matches_c_frame_payload() {
        assert_eq!(enc("identify offset=0 count=40").as_slice(), &[0x01, 0x00, 0x28]);
    }

    #[test]
    fn decode_identify_response() {
        let payload = [0x00, 0x00, 0x03, b'a', b'b', b'c'];
        let msgs: Vec<_> = D.decode(&payload).collect::<Result<_, _>>().unwrap();
        assert_eq!(msgs.len(), 1);
        assert_eq!(msgs[0].to_string(), "identify_response offset=0 data=\"abc\"");
        assert_eq!(msgs[0].raw, &payload);
    }

    #[test]
    fn roundtrip_every_format() {
        for fmt in D.formats() {
            let bufs: Vec<Vec<u8>> = (0..fmt.params.len()).map(|i| vec![i as u8; 3]).collect();
            let args: Vec<(&str, Arg)> = fmt
                .params
                .iter()
                .enumerate()
                .map(|(i, (n, t))| {
                    let a = match t {
                        ParamType::Buffer => Arg::Bytes(&bufs[i]),
                        ParamType::Byte => Arg::Int(200),
                        ParamType::U16 => Arg::Int(60000),
                        ParamType::I16 => Arg::Int(-30000),
                        ParamType::I32 => Arg::Int(-123456),
                        ParamType::U32 => Arg::Int(4_000_000_000),
                    };
                    (*n, a)
                })
                .collect();
            let mut b = MsgBuf::new();
            D.encode(fmt.name, &args, &mut b).unwrap();
            let dec: Vec<_> = D.decode(b.as_slice()).collect::<Result<_, _>>().unwrap();
            assert_eq!(dec.len(), 1, "{}", fmt.format);
            for ((n, a), (dn, da)) in args.iter().zip(dec[0].args()) {
                assert_eq!((*n, *a), (dn, da), "{}", fmt.format);
            }
            let owned = dec[0].to_owned();
            assert_eq!(owned.message().to_string(), dec[0].to_string());
        }
    }

    #[test]
    fn multiple_messages_in_one_payload() {
        let mut p = enc("get_uptime");
        p.extend(enc("identify offset=40 count=40").as_slice()).unwrap();
        let m: Vec<_> = D.decode(p.as_slice()).collect::<Result<_, _>>().unwrap();
        assert_eq!(m.iter().map(|m| m.name()).collect::<Vec<_>>(), ["get_uptime", "identify"]);
        assert_eq!(m[1].int("offset"), Some(40));
    }

    #[test]
    fn buffers_hex_and_text() {
        let b = enc("debug_ping data=0x0102ff");
        assert_eq!(b.as_slice(), &[D.by_name("debug_ping").unwrap().id as u8, 3, 1, 2, 0xff]);
        let t = enc("debug_ping data=hi");
        assert_eq!(&t.as_slice()[1..], &[2, b'h', b'i']);
    }

    #[test]
    fn errors() {
        let mut b = MsgBuf::new();
        assert_eq!(D.encode_str("nope", &mut b), Err(Error::UnknownMessage));
        assert_eq!(D.encode_str("identify offset=0", &mut b.clone()), Err(Error::MissingParam("count")));
        assert_eq!(D.encode_str("identify offset=x count=1", &mut MsgBuf::new()), Err(Error::BadValue("offset")));
        let mut big = MsgBuf::new();
        let long = format!("debug_ping data=0x{}", "00".repeat(60));
        assert!(matches!(D.encode_str(&long, &mut big), Err(Error::TooLong(_))));
        assert!(D.decode(&[0x7f]).next().unwrap().is_err()); // unknown id (-1)
        assert!(D.decode(&[0x01, 0x80]).next().unwrap().is_err()); // truncated
    }

    #[test]
    fn pins() {
        assert_eq!(pin_number("PE8", 32), Some(4 * 32 + 8));
        assert_eq!(pin_number("!PE7", 32), Some(4 * 32 + 7));
        assert_eq!(pin_number("PB6", 16), Some(22));
        assert_eq!(pin_number("PA16", 16), None);
        assert_eq!(pin_number("ADC_TEMPERATURE", 16), Some(254));
    }

    #[cfg(feature = "identify")]
    #[test]
    fn identify_blob_inflates() {
        let json = br#"{"commands":{"identify offset=%u count=%c":1},"responses":{"identify_response offset=%u data=%.*s":0}}"#;
        let z = miniz_oxide::deflate::compress_to_vec_zlib(json, 6);
        let d = Dictionary::from_identify(&z).unwrap();
        assert_eq!(d.len(), 2);
        assert_eq!(d.by_name("identify").unwrap().id, 1);
    }
}
