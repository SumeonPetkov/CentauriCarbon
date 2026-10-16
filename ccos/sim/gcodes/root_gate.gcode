; ROOT gate of the M8xxx commands (printer_para.cpp), for ccsim 06_gcode_parity.
; No motion: only config commands and their console output.
;LAYER:1
M8233 S0.31 P0 ; ignored: ROOT has not run in this file
ROOT
M8233 S0.32 P0 ; runs
UNROOT
M8233 S0.33 P0 ; ignored again
;LAYER:2
ECHO root gate done
