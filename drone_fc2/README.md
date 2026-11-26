# Mini drone firmware – `drone_fc2` (header-only)

Same flight controller as `drone_fc` — same behaviour, same Serial commands, **same UDP protocol as the Android app** —
written as **header-only** modules, plus a built-in **power / motor bench test** (`ptest`, BOOT button).

## 1. Open and flash
1. The folder must be named **`drone_fc2`** (same name as the `.ino`). After `git clone` it already is.
   If the IDE says *"the file needs to be inside a sketch folder named drone_fc2"*, rename the folder.
2. Arduino IDE → **File → Open** → `drone_fc2.ino` (the `.h`/`.cpp` files appear as tabs).
3. Board **XIAO_ESP32S3**, **USB CDC On Boot: Enabled**. Requires **ESP32 core 3.x** (uses `ledcAttach`).
4. Upload → open the Serial Monitor at 115200 baud, line ending **"New Line"** (or "Both NL & CR") to type commands.
5. Keep the board **still** for ~2 s after power-up (gyro calibration).

A healthy startup prints:
```
WiFi: "XIAO-Drone"  password "drone1234"  IP 192.168.4.1  UDP 4210
MPU WHO_AM_I = 0x70
MPU6500 found. Calibrating gyro: keep the drone STILL for ~2 s...
  Gyro calibrated. IMU ready.
```

**The XIAO always starts**, even without the MPU: motors are forced off first, then WiFi, the app link and Serial
come up. The MPU is searched for every second and calibrated in the background (the loop is never blocked).
Until the IMU is ready, **ARM is refused** and the app shows *Sensor NOT FOUND* or *Sensor calibrating*.
Moving the board during calibration simply restarts it. If the MPU stops answering while flying,
the drone disarms (motors off) and goes back to searching.

