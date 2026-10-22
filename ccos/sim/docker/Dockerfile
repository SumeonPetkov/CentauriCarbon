# ccsim (the ccos emulator) in a container, shown in a web browser.
#
# The window is drawn on a virtual X display (Xvfb), which noVNC serves on
# port 6080: open http://localhost:6080 and click the touchscreen there.
# Build from the repository root (the stock config files under firmware/
# are compiled in):
#
#   docker compose -f ccos/sim/docker/compose.yaml up --build
#
# See ccos/sim/docker/README.md.

FROM rust:1-slim-bookworm AS build
RUN apt-get update \
    && apt-get install -y --no-install-recommends pkg-config libx11-dev libxcursor-dev libxrandr-dev libxi-dev \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /src
COPY firmware/resources/e100/configs firmware/resources/e100/configs
COPY ccos ccos
WORKDIR /src/ccos
RUN --mount=type=cache,target=/usr/local/cargo/registry \
    --mount=type=cache,target=/src/ccos/target \
    cargo build --release -p ccsim \
    && cp target/release/ccsim /usr/local/bin/ccsim

FROM debian:bookworm-slim
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
       xvfb x11vnc novnc websockify \
       libx11-6 libxcursor1 libxrandr2 libxi6 libxext6 libxinerama1 \
    && rm -rf /var/lib/apt/lists/*
COPY --from=build /usr/local/bin/ccsim /usr/local/bin/ccsim
# test scripts, so `docker run … --window sim/scripts/10_bed_mesh.ccsim` works
COPY ccos/sim /ccos/sim
COPY ccos/sim/docker/entrypoint.sh /usr/local/bin/entrypoint.sh
# the opening page: the emulator at its real size, with zoom buttons
COPY ccos/sim/docker/index.html /usr/share/novnc/index.html
RUN sed -i 's/\r$//' /usr/local/bin/entrypoint.sh \
    && chmod +x /usr/local/bin/entrypoint.sh \
    && mkdir -p /gcodes /out
WORKDIR /ccos
EXPOSE 6080 6081
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["--speed", "3"]
