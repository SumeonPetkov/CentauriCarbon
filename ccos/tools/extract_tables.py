#!/usr/bin/env python3
"""Regenerate cc-proto/data/*.json from Elegoo's C++ host sources.

The Centauri Carbon host (firmware/core/klippy/msgproto.cpp) does NOT use the
Klipper data dictionary at runtime. Message ids, pin numbers and shutdown
strings are hard-coded tables that every MCU firmware (DSP, toolhead STM32,
strain-gauge STM32) is built to match. This script lifts those tables out so
other tools (cc-proto, docs) stay in sync with the C++ code.

Usage: python3 -I tools/extract_tables.py <repo-root>
"""
import json, os, re, sys, zlib

root = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(__file__), "..", "..")
src = open(os.path.join(root, "firmware/core/klippy/msgproto.cpp"), encoding="utf-8", errors="replace").read()

def block(signature):
    b = src[src.index(signature):]
    return b[:b.index("\n}\n")]

cmds = re.findall(r'messages_format\[(\d+)\]\s*=\s*"([^"]*)"',
                  block("void MessageParser::init_id_to_messages_format_map"))
table = {int(i): fmt.strip() for i, fmt in cmds}
shutdown = re.findall(r'shutdown_info\[(\d+)\]\s*=\s*"([^"]*)"',
                      block("void MessageParser::init_shutdown_info_map"))

out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "crates", "cc-proto", "data")
with open(os.path.join(out, "command_table.json"), "w") as f:
    json.dump({str(k): table[k] for k in sorted(table)}, f, indent=1, ensure_ascii=False)
with open(os.path.join(out, "static_strings.json"), "w") as f:
    json.dump({i: s for i, s in shutdown}, f, indent=1, ensure_ascii=False)

# The identify blobs compiled into the STM32 firmwares (stale, but useful to diff).
for board in ("extruder", "sg"):
    c = open(os.path.join(root, f"mcu/board/{board}/cmd/compile_time_request.c"), encoding="utf-8", errors="replace").read()
    m = re.search(r"command_identify_data\[\]\s*PROGMEM\s*=\s*\{(.*?)\};", c, re.S)
    raw = bytes(int(x, 0) for x in re.findall(r"0x[0-9a-fA-F]+|\b\d+\b", m.group(1)))
    d = json.loads(zlib.decompress(raw))
    with open(os.path.join(out, f"identify_{board}.json"), "w") as f:
        json.dump(d, f, indent=1, sort_keys=True)

print(f"{len(table)} commands, {len(shutdown)} static strings written to {os.path.normpath(out)}")
