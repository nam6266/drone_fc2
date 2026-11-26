# Drone controller app (Kotlin + Jetpack Compose)

## Open and run
1. Android Studio → **File → Open** → select `D:\work\drone\app` → wait for Gradle sync (the first one takes a while).
   - If Android Studio offers to upgrade AGP/Kotlin: accept, the code does not depend on specific versions.
2. Connect the phone (Android 10+, **USB debugging** on) → press **Run ▶**.
3. Power the XIAO (firmware `drone_fc`). Open the app → Android asks for the "Nearby devices"/location permission → **Allow** → dialog "Connect to XIAO-Drone" → **Connect**.
4. When the top bar shows **DISARMED** and **Battery x.xx V**, the app and the drone are talking.

Unit tests (right-click `app/src/test` → **Run Tests**): protocol, gesture detection, maneuvers, horizontal directions for each way of holding the phone.

## Screen (landscape)
```
┌ (MODE)   DISARMED  Battery 3.85 V · 72%  Chip 45 °C               [TAKE OFF] [LAND] ⚙ ┐
│                                                                                        │
│   ( left stick )             Power 0 %  Yaw 0             ( right stick )              │
│   FORWARD/BACK/LEFT/RIGHT         [ ARM ]                  FLY UP / FLY DOWN (centre = │
│   – in Gesture mode replaced      [ STOP ]                 hold height), ↺ ↻ yaw       │
│     by the gesture panel                                                               │
└────────────────────────────────────────────────────────────────────────────────────────┘
```

| Control | Behaviour |
|---|---|
| **MODE** (round button, top-left) | Tap it: two round buttons **Joystick** and **Gesture** stretch out of it like liquid drops (gooey effect, Android 12+; plain animation on older phones). Tap one to switch mode – the current mode has a white ring and ✓. Tap MODE (now ✕) again to close |
| **Left stick** (Joystick mode) | Drag in any direction: forward/back/left/right. Springs back to centre on release. Soft around centre (expo), 100 % at the edge |
| **Right stick** | Both axes **spring back to centre**. Vertical: up = **FLY UP**, down = **FLY DOWN**, centre = **hold height** (power = *Hover power*; up/down adds/removes up to *Up/Down strength*). Horizontal = **yaw** (turn the nose). Only active once airborne (after TAKE OFF). Relative drag: the knob does not jump to your finger |
| **ARM** | Only available with a signal. Motors stay off until TAKE OFF. If the drone refuses (tilted, low battery…) the app releases ARM and shows a message. While the top bar shows **Sensor NOT FOUND** (MPU wiring) or **Sensor calibrating** (keep the drone still ~2 s), ARM is blocked |
| **TEST** (top bar) | Ground **motor test** – see below. Only while disarmed |
| **STOP** | Motors off immediately, power to 0, cancels any automatic action |
| **TAKE OFF** | (Armed, landed) Power ramps up over 1.5 s to **Hover power** (set in ⚙). From then on the right stick controls height |
| **LAND** | Power drops to 80 % of Hover power → held for 2.5 s so the drone sinks slowly → ramps to 0 → disarms. **Use it when the drone is low (< 1 m)** |
| Pushing the right stick up during LAND | Cancels the landing and returns to hover power |

## Motor test (TEST button)
Spins motors **without the motion sensor, without arming and without stabilisation** – works on battery only,
even while the board is being moved or the MPU is missing. Use it to check wiring, spin direction, MOSFET heat
and how much the battery voltage sags under load.

1. Tap **TEST** → pick the motors (M1…M4) and the power.
2. **Press and HOLD** the big button: power ramps up in 0.5 s. **Release** = motors stop.
   The button turns green when the drone confirms the motors are running.
3. *Battery now* is live; *Lowest in test* keeps the lowest voltage seen. Green ≥ 3.6 V, amber 3.4–3.6 V,
   red < 3.4 V (the XIAO is close to resetting → check battery, wiring, bulk capacitor).

Safety: the drone stops the test by itself after 15 s, when the signal is lost, on STOP, or when the dialog
closes / the app goes to the background. **Remove the props or clamp the drone.**

## Gesture mode
Flick the phone horizontally and the drone moves **a short distance** in that direction:

| Gesture | Drone |
|---|---|
| Flick left / right | Moves left / right |
| Push away from you / pull toward you | Moves forward / back |
| **Slow** flick | ≈ 8 cm (set in ⚙) |
| **Fast** flick (hand > 0.70 m/s) | ≈ 30 cm (set in ⚙) |

- Hold the phone upright or flat: the app finds the horizontal plane from gravity, and **up/down motion is ignored**.
- The hand's opposite braking motion when it stops is **not** detected as a reverse gesture. After each gesture the app waits ~0.6 s and for the hand to be still.
- Only executed when **armed and airborne (after TAKE OFF)**. When not flying, the gesture panel still shows the arrow and the measured hand speed → **use it to practise and tune the threshold** before flying.
- New gestures are ignored while a move is running. Diagonal flicks (unclear direction) are ignored.

**Distances are only estimates**: the drone has no position sensor, so "move 30 cm" = tilt ~10° for ~0.4 s, then tilt the other way to brake. Real results may differ a lot (battery, drafts, drone lag) and the drone still drifts a little afterwards. Fly, measure, then adjust the cm values in ⚙.

## Settings (⚙) – stored on the phone
| Setting | Default | Meaning |
|---|---|---|
| Hover power | 45 % | Power that keeps the drone hovering = right stick centre. Find it in a test flight: if the drone keeps rising with the stick released, lower it; if it sinks, raise it |
| Up/Down strength | ± 15 % | Extra / less power with the right stick fully up / down |
| FAST flick | 30 cm | Estimated distance |
| SLOW flick | 8 cm | |
| FAST threshold | 0.70 m/s | Check the hand speed shown on the gesture panel to pick a threshold that suits you |

## Structure
| File | Purpose |
|---|---|
| `net/DroneProtocol.kt` | Encodes the 13-byte Command, decodes the 14-byte Telemetry — matches the firmware's `Protocol.h` |
| `net/DroneLink.kt` | Requests the WiFi, UDP socket bound to the drone network, sends at 50 Hz, receives telemetry |
| `sensor/MotionSensor.kt` | Reads gravity + linear acceleration sensors (100 Hz), only active in Gesture mode |
| `sensor/HorizontalFrame.kt` | Math: phone acceleration → horizontal "right / away" acceleration (relative to the screen) |
| `control/GestureDetector.kt` | Flick detection: direction + fast/slow, ignores the opposite braking motion |
| `control/Maneuver.kt` | A move: distance → tilt X° for T ms, then brake (t = √(d / g·tanθ)) |
| `DroneViewModel.kt` | Mode, joysticks, ARM/STOP, take-off/landing, gestures, settings |
| `ui/ControlScreen.kt` | Main screen |
| `ui/Joystick.kt` | Virtual joystick used on both sides |
| `ui/GesturePanel.kt`, `ui/SettingsDialog.kt` | Gesture panel, settings dialog |
| `ui/ModeMenu.kt` | Round MODE button that fans out Joystick / Gesture |
| `ui/Gooey.kt` | Gooey (metaball) RenderEffect: blur + alpha threshold, null below Android 12 |
| `MainActivity.kt` | Permissions, keeps the screen on, reports screen rotation to the sensor |

`control/` and `sensor/HorizontalFrame.kt` are plain Kotlin (no Android dependency), so they have unit tests in `app/src/test`.

## Built-in safety
- Leaving the app (Home button, screen lock) → sending stops → the drone enters **FAILSAFE** and descends. When you come back, arm/power are reset to 0, so the old flight command is not resent.
- If the drone disarms itself (crash, failsafe) → the app releases ARM and cancels any take-off/landing or move in progress.
- The screen stays on while the app is in use.

## Tests without motors
1. Flash the firmware, open the Serial Monitor, open the app.
2. Type `status` on Serial: the line **Link: YES**, the OK packet count rising ~50/s.
3. Press ARM (needs a battery, or temporarily set `ARM_MIN_VBAT = 0` on USB only) → the XIAO LED goes solid, the app shows **ARMED**.
4. TAKE OFF → `thr=` ramps up to ~450. Push the right stick up → `thr=` rises to ~600; release → back to ~450. Push the left stick right → `status` shows positive `r=`.
5. LAND → `thr=` ramps down, then DISARMED.
6. Gestures: tap **MODE** → **Gesture**, ARM, TAKE OFF, flick the phone left → the gesture panel shows **"Moving LEFT ~… cm"** in green. On Serial enable `log` (Serial Plotter): within less than a second the right motors (m1, m2) rise, then fall (braking), then return to equal.
7. Press Home → the LED blinks fast (FAILSAFE) → a few seconds later DISARMED.
