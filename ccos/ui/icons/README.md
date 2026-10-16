Icons used by the ccos UI. Most come from Elegoo's stock UI resources
(`firmware/resources/e100/images/<n>.png`, GPL-3.0 like the rest of this
repository): home=2, control=4, settings=5, tools=6, light=12, light_on=217,
nozzle=13, nozzle_hot=23, bed=14, bed_hot=24, fan=15, chamber=16, z_up=26, z_down=28.

Any PNG works. Single-colour icons are stored as 4-bit masks and can be
re-coloured in `ui/theme.toml` with `tint = "<colour>"`; multi-colour icons are
stored as RGBA. Nothing here is copied to the printer: icons are compiled into
the binary.
