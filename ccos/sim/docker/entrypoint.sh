#!/bin/sh
# Virtual display + VNC + noVNC (browser on :6080), then ccsim in its window.
# Arguments go to ccsim; the G-code folder is /gcodes (mount a folder there)
# and script screenshots go to /out.
set -e
export DISPLAY=:99
# ccsim's window: 2x LCD (960 x 544) + bed view (420) wide, + 120 px of state
Xvfb :99 -screen 0 1380x664x24 -nolisten tcp >/dev/null 2>&1 &
i=0
while [ ! -e /tmp/.X11-unix/X99 ] && [ $i -lt 50 ]; do sleep 0.1; i=$((i + 1)); done
x11vnc -display :99 -forever -shared -nopw -localhost -rfbport 5900 -quiet >/dev/null 2>&1 &
websockify --web /usr/share/novnc 6080 localhost:5900 >/dev/null 2>&1 &
echo "ccsim: open http://localhost:6080 in your browser"

case " $* " in
    *.ccsim*)
        # a test script: show it in the window
        exec ccsim --window --gcode-dir /gcodes --out /out "$@" ;;
    *)
        # the page's console and file panel talk to --control on :6081
        exec ccsim --gcode-dir /gcodes --control 0.0.0.0:6081 "$@" ;;
esac
