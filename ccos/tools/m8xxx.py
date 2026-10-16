"""Generate ccos's M8xxx table from Elegoo's printer_para.cpp.

usage: python m8xxx.py printer_para.cpp Define_default.h out_table.rs out_manual.txt
"""
import re, sys

src = open(sys.argv[1], encoding="utf-8", errors="replace").read()
defs_src = open(sys.argv[2], encoding="utf-8", errors="replace").read()
out_rs, out_other = sys.argv[3], sys.argv[4]

defines = {}
for k, v in re.findall(r'#define\s+(DEFAULT_\w+)\s+(.+?)\s*$', defs_src, re.M):
    defines[k] = re.sub(r'\s*//.*$', '', v).strip()

def num(expr):
    e = expr
    for _ in range(3):
        e = re.sub(r'DEFAULT_\w+', lambda m: '(' + defines.get(m.group(0), 'nan') + ')', e)
    e = re.sub(r'(\d)[fF]\b', r'\1', e)
    try:
        return float(eval(e, {"nan": float("nan")}))
    except Exception:
        return None

def strval(expr):
    v = defines.get(expr, expr).strip()
    return v.strip('"')

def rs_f(v):
    if v is None or v != v:
        return "f64::NAN"
    s = repr(float(v))
    return s

funcs = re.findall(r'void Printer_para::cmd_(M8\d{3})\(GCodeCommand &gcode\)\s*\{(.*?)\n\}', src, re.S)
gate = 'if (!is_root && Printer::GetInstance()->m_virtual_sdcard->is_cmd_from_sd()) { return; }'
inv_gate = 'if (!is_root && !Printer::GetInstance()->m_virtual_sdcard->is_cmd_from_sd()) { return; }'
GI = r'Printer::GetInstance\(\)->'
effects = [
    (r'm_tool_head->m_kin->m_rails\[(\d)\]->m_position_endstop = \w+;', 'RailEndstop({})'),
    (r'm_tool_head->m_kin->m_rails\[(\d)\]->m_position_max = \w+;', 'RailMax({})'),
    (r'm_tool_head->m_kin->m_rails\[(\d)\]->m_homing_speed = \w+;', 'RailHomingSpeed({})'),
    (r'm_tool_head->m_kin->m_rails\[(\d)\]->m_second_homing_speed = \w+;', 'RailSecondSpeed({})'),
    (r'm_tool_head->m_kin->m_rails\[(\d)\]->m_homing_retract_speed = \w+;', 'RailRetractSpeed({})'),
    (r'm_tool_head->m_kin->m_rails\[(\d)\]->m_homing_retract_dist = \w+;', 'RailRetractDist({})'),
    (r'm_printer_extruder->m_heater->m_min_temp = \w+;', 'HeaterMinTemp(0)'),
    (r'm_printer_extruder->m_heater->m_max_temp = \w+;', 'HeaterMaxTemp(0)'),
    (r'm_bed_heater->m_heater->m_min_temp = \w+;', 'HeaterMinTemp(1)'),
    (r'm_bed_heater->m_heater->m_max_temp = \w+;', 'HeaterMaxTemp(1)'),
    (r'm_gcode_io->single_command\("SET_VELOCITY_LIMIT VELOCITY=" \+ to_string\(\w+\)\);', 'Script("SET_VELOCITY_LIMIT VELOCITY=")'),
    (r'm_gcode_io->single_command\("SET_VELOCITY_LIMIT ACCEL=" \+ to_string\(\w+\)\);', 'Script("SET_VELOCITY_LIMIT ACCEL=")'),
    (r'm_gcode_io->single_command\("SET_VELOCITY_LIMIT SQUARE_CORNER_VELOCITY=" \+ to_string\(\w+\)\);', 'Script("SET_VELOCITY_LIMIT SQUARE_CORNER_VELOCITY=")'),
    (r'm_gcode_io->single_command\("SET_PRESSURE_ADVANCE ADVANCE=" \+ to_string\(\w+\)\);', 'Script("SET_PRESSURE_ADVANCE ADVANCE=")'),
    (r'm_tool_head->m_kin->m_max_z_velocity = \w+;', 'MaxZVelocity'),
    (r'm_tool_head->m_kin->m_max_z_accel = \w+;', 'MaxZAccel'),
    (r'm_resonance_tester->m_test->m_\w+ = \w+;', 'None'),
]

rows, other = [], []
for name, body in funcs:
    b = re.sub(r'//[^\n]*', '', body)
    b = re.sub(r'\s+', ' ', b).strip()
    gating = 'Gated'
    if gate in b:
        b = b.replace(gate, '')
    elif inv_gate in b:
        b = b.replace(inv_gate, '')
        gating = 'Inverted'
    else:
        gating = 'Ungated'
    b = b.replace('DisErrorMsg();', '').strip()
    m = re.fullmatch(r'std::string ori_data = (\w+); switch \(gcode\.get_int\("S", INT_MIN\)\) \{ case 1: set_cfg\("([^"]+)", "([^"]+)", ori_data\); break; case -1: set_cfg\("[^"]+", "[^"]+", "!" \+ ori_data\); break; default: break; \};?', b)
    if m:
        rows.append((name, f'Kind::Pin("{strval(m.group(1))}")', m.group(2), m.group(3), 'None', gating))
        continue
    m = re.fullmatch(r'set_cfg\("([^"]+)", "([^"]+)", gcode\.get_double\("S", ' + GI + r'm_pconfig->GetDouble\("([^"]+)", "([^"]+)", (.+?)\)\)\);', b)
    if m:
        rows.append((name, f'Kind::Double({rs_f(num(m.group(5)))}, "{m.group(3)}", "{m.group(4)}")', m.group(1), m.group(2), 'None', gating))
        continue
    m = re.fullmatch(r'(double|int) (\w+) = gcode\.get_(double|int)\("S", ' + GI + r'm_pconfig->Get(Double|Int)\("([^"]+)", "([^"]+)", (.+?)\)\); set_cfg\("([^"]+)", "([^"]+)", (?:\2|gcode\.get_double\("S", [^;]+\))\);(.*)', b)
    if m:
        decl, var, getter, cfgget, rsec, rkey, dflt, sec, key, rest = m.groups()
        rest = rest.strip()
        eff = 'None'
        if rest:
            for pat, tmpl in effects:
                mm = re.fullmatch(GI + pat, rest)
                if mm:
                    eff = tmpl.format(*mm.groups()) if mm.groups() else tmpl
                    break
            else:
                other.append((name, gating, body))
                continue
        d = num(dflt)
        if decl == 'int' and getter == 'double':
            kind = f'Kind::DoubleAsInt({rs_f(d)}, "{rsec}", "{rkey}")'
        elif decl == 'int':
            kind = f'Kind::Int({int(d) if d == d and d is not None else 0}, "{rsec}", "{rkey}")'
        else:
            kind = f'Kind::Double({rs_f(d)}, "{rsec}", "{rkey}")'
        rows.append((name, kind, sec, key, f'Effect::{eff}', gating))
        continue
    m = re.fullmatch(r'std::string (\w+) = gcode\.get_string\("S", ' + GI + r'm_pconfig->GetString\("([^"]+)", "([^"]+)", (\w+)\)\); set_cfg\("([^"]+)", "([^"]+)", \1\);', b)
    if m:
        rows.append((name, f'Kind::Str("{strval(m.group(4))}", "{m.group(2)}", "{m.group(3)}")', m.group(5), m.group(6), 'None', gating))
        continue
    other.append((name, gating, body))

with open(out_rs, "w", newline="\n", encoding="utf-8") as f:
    f.write("// @generated by ccos/tools/m8xxx.py from firmware/core/klippy/printer_para.cpp\n")
    f.write("// (name, value kind with its default, config section, key, runtime effect, gate)\n")
    f.write("pub const TABLE: &[(&str, Kind, &str, &str, Effect, Gate)] = &[\n")
    for name, kind, sec, key, eff, gating in rows:
        if not eff.startswith('Effect::'):
            eff = 'Effect::' + eff
        f.write(f'    ("{name}", {kind}, "{sec}", "{key}", {eff}, Gate::{gating}),\n')
    f.write("];\n")

def cfg_text(macro):
    """What set_cfg() stores for a DEFAULT_ macro (string / int / double overload)."""
    v = defines.get(macro, '').strip()
    if v.startswith('"'):
        return v.strip('"')
    if re.fullmatch(r'-?\d+', v):
        return str(int(v))
    if v in ('true', 'false'):
        return '1' if v == 'true' else '0'
    d = num(v)
    return f"{d:.6f}"

body = re.search(r'void Printer_para::load_default_config\(\)\s*\{(.*?)\n\}', src, re.S).group(1)
with open(out_rs, "a", newline="\n", encoding="utf-8") as f:
    f.write("\n/// `load_default_config` (M8801): what it writes, misspellings included.\n")
    f.write("pub const DEFAULT_CONFIG: &[(&str, &str, &str)] = &[\n")
    for sec, key, macro in re.findall(r'set_cfg\("([^"]+)", "([^"]+)", (\w+)\);', body):
        f.write(f'    ("{sec}", "{key}", "{cfg_text(macro)}"),\n')
    f.write("];\n")

with open(out_other, "w", newline="\n", encoding="utf-8") as f:
    for name, gating, body in other:
        f.write(f"=== {name} {gating}\n{body.strip()}\n\n")
print(len(rows), "table rows,", len(other), "manual:", " ".join(o[0] for o in other))
