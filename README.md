# ardupilot_connect

A small C++20 ground-control client that talks to an **ArduPilot** autopilot over **MAVLink v2**, written from scratch on top of raw POSIX sockets and the header-only `c_library_v2`.

This is a learning project. The goal is to understand MAVLink at the wire level (framing, sequence numbers, system/component IDs, command/acknowledge flow) and to build a clean, layered architecture that can later move from UDP to a UART link on a microcontroller.

> Developed and tested against **ArduCopter SITL** (software-in-the-loop). Do not point it at a real vehicle.

---

## What it does today

- Listens for MAVLink traffic over UDP and parses it byte by byte
- Finds the autopilot from its `HEARTBEAT` (ignoring heartbeats with `MAV_AUTOPILOT_INVALID`, i.e. other ground stations) and from then on accepts messages only from that `sysid`/`compid`
- Keeps the vehicle's state in a `VehicleState` struct:
  - `HEARTBEAT` → flight mode (`custom_mode`), armed flag, system status, time of last heartbeat
  - `GLOBAL_POSITION_INT` → latitude/longitude (deg), relative altitude (m), climb rate (m/s)
- Decodes `COMMAND_ACK` and ignores acknowledgements addressed to other ground stations (e.g. MAVProxy) by checking `target_system`
- Identifies itself as a GCS by sending its own `HEARTBEAT` at 1 Hz (`sysid 254`, `MAV_COMP_ID_MISSIONPLANNER`)
- Sends `COMMAND_LONG` to the discovered vehicle — currently switches it to **GUIDED** mode once and confirms the result via `COMMAND_ACK`

Example output:

```
mode STABILIZE | DISARMED | status STANDBY
ACK command 176 result ACCEPTED
mode GUIDED    | DISARMED | status STANDBY
```

---

## Architecture

Each layer knows only about the one directly below it:

```
            ┌──────────────────┐
  bytes ──► │    UdpServer     │  transport: bind, recv (remembers sender), send, poll
            └────────┬─────────┘
                     │ std::span<const uint8_t>
            ┌────────▼─────────┐
            │  MavlinkParser   │  byte-by-byte framing via mavlink_parse_char,
            └────────┬─────────┘  callback per complete message
                     │ const mavlink_message_t&
            ┌────────▼─────────┐
            │ TelemetryHandler │  filter by source, dispatch per msgid
            └────────┬─────────┘  (onHeartbeat / onCommandAck / onGlobalPositionInt)
                     │ const VehicleState&
            ┌────────▼─────────┐
            │      main        │  control loop: timers, decides what to send
            └────────┬─────────┘
                     │
            ┌────────▼─────────┐
            │  MavlinkSender   │  pack HEARTBEAT / COMMAND_LONG, serialize, send
            └──────────────────┘
```

Design notes:

- **The transport knows nothing about MAVLink, and the parser knows nothing about UDP.** The same parser is meant to work unchanged on a serial stream later.
- **The parser is fed bytes, not packets.** `mavlink_parse_char` is a state machine, so a message split across reads (typical for UART) or several messages in one datagram are both handled.
- **Telemetry is state, not logs.** `TelemetryHandler` updates a single `VehicleState`; the rest of the program reads it through `state()` instead of reacting to individual messages.
- **One link = one MAVLink channel.** The parser and the sender for the same link share a channel (`MAVLINK_COMM_0`), which keeps the incoming parse state and the outgoing `seq` counter per link.
- **The UDP socket is a server.** MAVProxy's `--out=udp:host:port` *sends* to us, so we `bind()` and reply to whoever wrote last.
- **Non-blocking socket + `poll()` with timeout**, so the main loop can also run timers (heartbeat, command timeouts) without busy-waiting.
- Errors from system calls are reported as exceptions carrying `strerror(errno)`; "no data yet" (`EAGAIN`, `EINTR`) is not an error.

---

## Project layout

```
ardupilot_connect/
├── CMakeLists.txt
├── README.md
├── include/
│   ├── transport/
│   │   └── udp_server.hpp          # UDP socket: bind / recv / send / wait_readable
│   ├── mavlink/
│   │   ├── mavlink_parser.hpp      # bytes → mavlink_message_t (callback)
│   │   ├── mavlink_sender.hpp      # HEARTBEAT, COMMAND_LONG
│   │   └── mavlink_utils.hpp       # enum → string helpers for logging
│   └── vehicle/
│       ├── vehicle_state.hpp       # VehicleState struct
│       └── telemetry_handler.hpp   # message dispatch, updates VehicleState
└── src/
    ├── main.cpp
    ├── transport/
    │   └── udp_server.cpp
    ├── mavlink/
    │   ├── mavlink_parser.cpp
    │   ├── mavlink_sender.cpp
    │   └── mavlink_utils.cpp
    └── vehicle/
        └── telemetry_handler.cpp
```

All sources except `main.cpp` are built into a static library (`mav_listener_lib`), so they can be linked into tests later without the executable.

---

## Requirements

- C++20 compiler (tested with AppleClang on macOS, arm64)
- CMake ≥ 3.16, Ninja (optional)
- [MAVLink C library v2](https://github.com/mavlink/c_library_v2) (header-only, `ardupilotmega` dialect)
- [ArduPilot](https://github.com/ArduPilot/ardupilot) SITL for testing

---

## Build

Get the MAVLink headers:

```bash
git clone https://github.com/mavlink/c_library_v2.git
```

Configure and build, pointing CMake at the headers:

```bash
cmake -S . -B build -G Ninja -DMAVLINK_DIR=/path/to/c_library_v2
cmake --build build
```

`MAVLINK_DIR` defaults to `~/www/cpp_embedded/libs/mavlink/c_library_v2`; configuration fails with a clear message if `ardupilotmega/mavlink.h` is not found there. `compile_commands.json` is exported for editor tooling (clangd, VS Code CMake Tools).

---

## Running against ArduPilot SITL

### Same machine

```bash
# terminal 1 — simulator with an extra MAVLink output for this program
cd ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter --console --map --out=udp:127.0.0.1:14551

# terminal 2
./build/mav_listener
```

### SITL in a Linux VM, client on the host (e.g. UTM on macOS)

SITL runs inside an Ubuntu VM; this program runs on the macOS host. With UTM's *Shared Network* mode the host is usually reachable from the VM at the default gateway address:

```bash
# inside the VM: find the host address ("via ...")
ip route | grep default

# inside the VM: send MAVLink to the host
sim_vehicle.py -v ArduCopter --console --map --out=udp:192.168.64.1:14551
```

Then run `./build/mav_listener` on the host. Allow incoming connections if the macOS firewall asks.

### Useful checks

```bash
# is UDP traffic reaching the port at all?
echo "hello" | nc -u -w1 127.0.0.1 14551

# watch packets in both directions (on macOS try -i bridge100 if -i any is noisy)
sudo tcpdump -i any -n udp port 14551
```

A 21-byte packet from port 14551 once per second is this program's own heartbeat.

---

## Roadmap

- [x] UDP server transport with sender tracking
- [x] Byte-wise MAVLink parser with per-message callback
- [x] GCS heartbeat at 1 Hz
- [x] `COMMAND_LONG` + `COMMAND_ACK` handling (set GUIDED mode)
- [x] `VehicleState` + `TelemetryHandler` with per-message handlers
- [x] `GLOBAL_POSITION_INT` decoding (position, relative altitude, climb rate)
- [ ] `STATUSTEXT` and `ATTITUDE` decoding
- [ ] Flight sequence as a state machine: GUIDED → ARM → TAKEOFF → climb → LAND → disarmed
- [ ] Pending-command tracking with timeouts, retries (`confirmation` field) and per-result handling
- [ ] Heartbeat-loss detection based on `last_heartbeat`
- [ ] Transport interface (`ITransport`) so UDP and serial are interchangeable
- [ ] Serial transport and a MAVLink node on an **STM32 Nucleo-F411RE**
- [ ] PX4 SITL support (different `custom_mode` encoding, Offboard mode)
- [ ] Unit tests for the parser and the state machine

---

## Notes and pitfalls learned along the way

- MAVLink v2 trims trailing zero bytes of the payload, so the same message type can arrive with different lengths.
- `HEARTBEAT.custom_mode` is autopilot- and vehicle-specific: the ArduCopter mode table differs from ArduPlane and from PX4.
- Heartbeats are sent by ground stations too; filter by `autopilot != MAV_AUTOPILOT_INVALID` to find the vehicle.
- MAVProxy forwards *all* traffic to every output, including acknowledgements of its own commands — check `COMMAND_ACK.target_system`.
- `GLOBAL_POSITION_INT` uses integer units: `lat`/`lon` in degE7, `relative_alt` in mm, `vz` in cm/s and **positive down** (NED), so climb rate is `-vz`.
- ArduCopter disarms automatically if it is armed but does not take off within a few seconds (`DISARM_DELAY`).
- `STATUSTEXT.text` is a fixed `char[50]` and is not guaranteed to be null-terminated.

---

## License

MIT — see [LICENSE](LICENSE).