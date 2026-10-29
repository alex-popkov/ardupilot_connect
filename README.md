# ardupilot_connect

A small C++20 ground-control client that talks to an **ArduPilot** autopilot over **MAVLink v2**, written from scratch on top of raw POSIX sockets and the header-only `c_library_v2`.

This is a learning project. The goal is to understand MAVLink at the wire level (framing, sequence numbers, system/component IDs, command/acknowledge flow) and to build a clean, layered architecture that can later move from UDP to a UART link on a microcontroller.

> Developed and tested against **ArduCopter SITL** (software-in-the-loop). Do not point it at a real vehicle.

---

## What it does today

Started against a freshly booting SITL, the program flies a complete mission on its own:

**wait for the autopilot → GUIDED → arm → take off to 10 m → land → disarmed**

Along the way it:

- Listens for MAVLink over UDP and parses it byte by byte
- Finds the autopilot from its `HEARTBEAT` (ignoring `MAV_AUTOPILOT_INVALID`, i.e. other ground stations) and from then on accepts telemetry only from that `sysid`/`compid`
- Keeps the vehicle's state in a `VehicleState` struct (mode, armed flag, system status, position, relative altitude, climb rate)
- Prints `STATUSTEXT` messages from the autopilot with their severity, so the reason behind every refusal is visible
- Identifies itself as a GCS with its own 1 Hz `HEARTBEAT` (`sysid 254`)
- Drives the flight with a state machine that:
  - waits until the autopilot has finished booting (`MAV_STATE_STANDBY`) before sending anything
  - resends a command if no `COMMAND_ACK` arrives (with the `confirmation` counter), and gives up after a few silent attempts
  - retries temporarily refused commands after a pause — e.g. arming while the EKF and GPS are still initialising
  - fails fast on refusals that will not change (`DENIED`, `UNSUPPORTED`, ...)
  - confirms each step through telemetry, not just the ACK (e.g. waits for the heartbeat to show GUIDED / ARMED)
  - bounds every step with its own time limit
- Keeps reading for a couple of seconds after finishing, so late `STATUSTEXT` messages are not lost

Example output (cold start of SITL, shortened):

```
mode STABILIZE | DISARMED | status BOOT
[autopilot sev=INFO] ArduPilot Ready
mode STABILIZE | DISARMED | status STANDBY
[seq] WAIT_VEHICLE -> SET_GUIDED
mode GUIDED    | DISARMED | status STANDBY
[seq] SET_GUIDED -> ARMING
[seq] rejected: FAILED, retry in 3 s
[autopilot sev=CRITICAL] Arm: EKF attitude is bad
...
[autopilot sev=CRITICAL] Arm: Need Position Estimate
[autopilot sev=INFO] EKF3 IMU0 is using GPS
[seq] command 400: retrying after rejection
[autopilot sev=INFO] Arming motors
mode GUIDED    | ARMED    | status STANDBY
[seq] ARMING -> TAKEOFF
[seq] TAKEOFF -> CLIMBING
mode GUIDED    | ARMED    | status ACTIVE
[seq] CLIMBING -> LANDING
mode LAND      | ARMED    | status ACTIVE
[autopilot sev=INFO] Disarming motors
mode LAND      | DISARMED | status STANDBY
[seq] LANDING -> DONE
```

---

## Architecture

```
            ┌──────────────────┐
  bytes ──► │    UdpServer     │  transport: bind, recv (remembers sender), send, poll
            └────────┬─────────┘
                     │ bytes
            ┌────────▼─────────┐
            │  MavlinkParser   │  byte-by-byte framing via mavlink_parse_char
            └────────┬─────────┘
                     │ mavlink_message_t
          ┌──────────┴───────────┐
┌─────────▼────────┐   ┌─────────▼────────┐
│ TelemetryHandler │   │  FlightSequence  │  state machine: reacts to COMMAND_ACK
│ → VehicleState   │──►│  (reads state)   │  and to telemetry, owns retries/timeouts
└──────────────────┘   └─────────┬────────┘
                                 │ commands
                       ┌─────────▼────────┐
                       │  MavlinkSender   │  pack HEARTBEAT / COMMAND_LONG, serialize, send
                       └──────────────────┘
```

`main` only wires the pieces together: it receives, feeds each message to `TelemetryHandler` (first) and `FlightSequence`, runs the heartbeat timer and calls `FlightSequence::tick()` on every loop iteration.

Design notes:

- **The transport knows nothing about MAVLink, and the parser knows nothing about UDP.** The same parser is meant to work unchanged on a serial stream later.
- **The parser is fed bytes, not packets.** A message split across reads (typical for UART) or several messages in one datagram are both handled.
- **Telemetry is state; `COMMAND_ACK` is a reply.** `TelemetryHandler` builds the model of the vehicle that anyone can read. Acknowledgements only make sense to whoever sent the command, so they are handled by the state machine.
- **One place changes the state.** `FlightSequence::setState()` is the only function that assigns the state; it also clears the active command, sets the step's deadline and performs the entry action (sending the step's command).
- **Two kinds of transitions.** Some steps finish on a `COMMAND_ACK`, others on telemetry (`tick()` checks mode, armed flag, altitude). Steps whose effect can be observed are confirmed by telemetry.
- **Invalid states are unrepresentable.** The active command is a single `std::optional<ActiveCommand>` with an explicit phase (`AwaitingAck` / `WaitingRetry`) and one `deadline`, instead of several loosely related fields.
- **Retry policy is a pure function.** `classifyAck(command, result)` maps an ACK to `Accept / InProgress / RetryLater / Fail` in one place.
- **One link = one MAVLink channel.** The parser and the sender for the same link share a channel (`MAVLINK_COMM_0`), which keeps the incoming parse state and the outgoing `seq` counter per link.
- **The UDP socket is a server.** MAVProxy's `--out=udp:host:port` *sends* to us, so we `bind()` and reply to whoever wrote last.
- **Non-blocking socket + `poll()` with timeout**, so the main loop can run timers (heartbeat, command timeouts, step deadlines) without busy-waiting.
- Errors from system calls are reported as exceptions carrying `strerror(errno)`; "no data yet" (`EAGAIN`, `EINTR`) is not an error.

---

## The flight state machine

| State | Entry action | Leaves when |
|---|---|---|
| `WAIT_VEHICLE` | — | autopilot found **and** `system_status == STANDBY` |
| `SET_GUIDED` | `MAV_CMD_DO_SET_MODE` (GUIDED) | ACK accepted **and** heartbeat shows GUIDED |
| `ARMING` | `MAV_CMD_COMPONENT_ARM_DISARM` | ACK accepted **and** heartbeat shows ARMED |
| `TAKEOFF` | `MAV_CMD_NAV_TAKEOFF` (param7 = altitude) | ACK accepted |
| `CLIMBING` | — | relative altitude within 0.5 m of the target |
| `LANDING` | `MAV_CMD_DO_SET_MODE` (LAND) | ACK accepted **and** vehicle disarmed |
| `DONE` / `FAILED` | — | terminal |

Command handling:

| Situation | Reaction |
|---|---|
| no ACK within 1 s | resend with `confirmation + 1`; `FAILED` after 3 silent attempts |
| `ACCEPTED` | stop waiting; move on (or let telemetry confirm the step) |
| `IN_PROGRESS` | keep waiting, extend the deadline |
| `TEMPORARILY_REJECTED`, or `FAILED` for arming | retry after 3 s |
| anything else | `FAILED` |
| any step exceeds its time limit | `FAILED` |

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
│   │   ├── mavlink_parser.hpp      # bytes → mavlink_message_t
│   │   ├── mavlink_sender.hpp      # HEARTBEAT, COMMAND_LONG
│   │   └── mavlink_utils.hpp       # enum → string helpers for logging
│   └── vehicle/
│       ├── vehicle_state.hpp       # VehicleState struct
│       ├── telemetry_handler.hpp   # message dispatch, updates VehicleState
│       └── flight_sequence.hpp     # flight state machine
└── src/
    ├── main.cpp
    ├── transport/
    │   └── udp_server.cpp
    ├── mavlink/
    │   ├── mavlink_parser.cpp
    │   ├── mavlink_sender.cpp
    │   └── mavlink_utils.cpp
    └── vehicle/
        ├── telemetry_handler.cpp
        └── flight_sequence.cpp
```

All sources except `main.cpp` are built into a static library (`mav_listener_lib`), so they can be linked into tests later without the executable.

---

## Requirements

- C++20 compiler (tested with AppleClang on macOS, arm64)
- CMake ≥ 3.16, Ninja (optional)
- [MAVLink C library v2](https://github.com/mavlink/c_library_v2) (header-only, `ardupilotmega` dialect)
- [ArduPilot](https://github.com/ArduPilot/ardupilot) SITL for testing (tested with ArduCopter 4.8.0-dev)

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

The program can be started at any time — even while SITL is still booting. It waits for the autopilot and retries arming until the pre-arm checks pass (on a cold start this took roughly 45–50 s in testing).

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
- [x] Byte-wise MAVLink parser
- [x] GCS heartbeat at 1 Hz
- [x] `COMMAND_LONG` + `COMMAND_ACK` handling
- [x] `VehicleState` + `TelemetryHandler` with per-message handlers
- [x] `GLOBAL_POSITION_INT` decoding (position, relative altitude, climb rate)
- [x] `STATUSTEXT` decoding with severity names
- [x] Flight sequence as a state machine: GUIDED → ARM → TAKEOFF → climb → LAND → disarmed
- [x] Command tracking with timeouts, retries (`confirmation` field) and per-result policy
- [x] Per-step time limits; steps confirmed by telemetry
- [ ] `CommandClient`: move command sending, ACK matching and retries out of `FlightSequence`
- [ ] Unit tests for `classifyAck` and the state machine (no network needed)
- [ ] `ATTITUDE` decoding
- [ ] Heartbeat-loss detection based on `last_heartbeat`
- [ ] Multi-waypoint flight in GUIDED (`SET_POSITION_TARGET_GLOBAL_INT`) or via the mission protocol
- [ ] Transport interface (`ITransport`) so UDP and serial are interchangeable
- [ ] Serial transport and a MAVLink node on an **STM32 Nucleo-F411RE**
- [ ] PX4 SITL support (different `custom_mode` encoding, Offboard mode)

---

## Notes and pitfalls learned along the way

**Protocol**

- MAVLink v2 trims trailing zero bytes of the payload, so the same message type can arrive with different lengths.
- `HEARTBEAT.custom_mode` is autopilot- and vehicle-specific: the ArduCopter mode table differs from ArduPlane and from PX4.
- Heartbeats are sent by ground stations too; filter by `autopilot != MAV_AUTOPILOT_INVALID` to find the vehicle.
- MAVProxy forwards *all* traffic to every output, including acknowledgements of its own commands — check `COMMAND_ACK.target_system`.
- `GLOBAL_POSITION_INT` uses integer units: `lat`/`lon` in degE7, `relative_alt` in mm, `vz` in cm/s and **positive down** (NED), so climb rate is `-vz`. Divide by `1e7` / `1000.0`, not by integers.
- `STATUSTEXT.text` is a fixed `char[50]` and is not guaranteed to be null-terminated — print it with `%.*s`.

**Commanding ArduCopter**

- **An ACK is not a lasting state.** A mode change accepted while the autopilot was still booting was silently reverted to STABILIZE once it finished initialising; arming then succeeded, and takeoff was refused. Wait for `MAV_STATE_STANDBY` and confirm results through telemetry.
- ArduCopter answers a failed pre-arm check with `MAV_RESULT_FAILED`, even when the cause is temporary (EKF not converged, no GPS fix). Treating it as retryable for arming is what makes a cold start work.
- The reason for a refusal arrives as a separate `STATUSTEXT`, sometimes *after* the `COMMAND_ACK` — don't exit immediately on failure.
- ArduCopter disarms automatically if it is armed but does not take off within a few seconds (`DISARM_DELAY`), so ARM → TAKEOFF must follow quickly.
- `NAV_TAKEOFF` is only accepted in GUIDED; the target altitude goes in `param7`.
- Landing from 10 m in LAND mode takes ~20 s, plus a few seconds of touchdown detection before the automatic disarm.

**C++**

- A periodic timer advanced with `next += period` bursts to "catch up" after a long pause — reset it when it falls behind.
- Passing a whole array to `std::span` instead of `{buf, n}` silently feeds stale bytes to the parser.
- Fields that depend on the current state must be assigned on *every* state entry, including the "no value" branch.

---

## License

MIT — see [LICENSE](LICENSE).