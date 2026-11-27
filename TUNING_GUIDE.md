# BIOBUZZ tuning guide

Do these in order. Each step builds on the one before it.

| # | What | OpMode / tool | Result goes into |
|---|---|---|---|
| 0 | Hardware check | `TEST: Flywheel Encoder`, `TUNE: Servo Positions` | `subsystems/Shooter.java` |
| 1 | Pedro: drivetrain + odometry | AutoTune web page | `pedro/Constants.java` |
| 2 | Pedro: Foresight (path following) | AutoTune web page | `pedro/Constants.java` |
| 3 | Flywheel PIDF | `TUNE: Flywheel PIDF` | `subsystems/FlywheelPIDF.java` |
| 4 | Shooting LUT | `TUNE: LUT Finder (start 72,72)` | `subsystems/Shooter.java` |
| 5 | Check everything | `TEST: TeleOp (start 72,72)`, then the autos | |

**Values changed with the gamepad are lost when the Robot Controller app restarts.**
Always copy the final numbers into the Java file, then rebuild and install (Run ▶ in Android Studio).

Use a **charged battery (above 13 V)** for all tuning. Flywheel speed, Foresight and LUT values all change with battery voltage.

---

## 0. Hardware check

The Driver Hub config names are in [PEDRO3_NOTES.md](PEDRO3_NOTES.md#driver-hub-robot-configuration-names).

**TEST: Flywheel Encoder**
- Spin the flywheel by hand, or hold the right trigger to power it.
- Velocity must go **positive** when the wheel spins the shooting direction. If it goes negative, flip `Shooter.FLYWHEEL_DIR`. If it stays 0, the encoder cable is loose or in the wrong port.
- Hold the trigger fully and write down the top velocity (call it `Vmax`). You need it in step 3.

**TUNE: Servo Positions**
- D-pad left/right picks the servo. D-pad up/down moves it by 0.01, bumpers by 0.05.
- Find gate open and closed, then put them in `Shooter.GATE_OPEN` and `GATE_CLOSE`.
- Check the hood's usable range (the lowest and highest positions that don't bind).

---

## 1. Pedro: drivetrain + odometry (AutoTune)

AutoTune is a web page hosted by the robot. It runs the robot by itself: you don't press anything on the Driver Hub, and **the robot can move as soon as you click**.

1. Install the code, turn the robot on, and connect a laptop or phone to the robot's Wi-Fi.
2. Open **http://192.168.43.1:10158** in a browser.
3. Put the robot on the field with **at least 2 tiles of space in every direction**. Keep someone at the Driver Hub: the tuner shows there as a running OpMode ("Pedro Tuning"), so STOP should end it.

### 1a. Mecanum Tuner (*1. Mecanum Tuner*)
- Enter the motor names: `fl`, `fr`, `bl`, `br`.
- Each wheel spins on its own. Answer whether it spun and which way (the diagram shows "forward").
- It prints a Java block. Compare it with `drivetrainConfig` in `pedro/Constants.java` and copy any direction that differs.

### 1b. Pinpoint Tuner (*2. Pinpoint Tuner*)
- Setup: name `pinpoint`, pod type **4-bar**.
- **Push distance:** it asks how far you'll push the robot (default 48 in). Use a tape measure, push it straight forward exactly that far, and confirm.
- It then finds the forward and strafe directions and the pod offsets. For the offsets the robot spins in place, so give it room.
- Paste the printed block over `localizerConfig` in `pedro/Constants.java`. It should be close to the current values (x offset ≈ 3.0, y offset ≈ −5.7, x pod FORWARD, y pod REVERSED).

### 1c. Check localization before going further (*4. Tests*)
Rebuild and install first if you changed `Constants.java`.
- **Localization / Pose Test:** drive or push the robot. Forward must increase **x**, moving left must increase **y**, and turning left (counter-clockwise) must increase heading.
- **Odometry Test:** it must pass. Don't tune Foresight on bad odometry.
- You can also use **TEST: TeleOp (start 72,72)**. Its POSITION block shows x, y, heading and "moved from start". Push the robot exactly 24 in forward: dx (or dy, depending on heading) should read about 24.

> Don't tune odometry anywhere the floor moves (a train, a bus): the gyro picks up that motion.

---

## 2. Pedro: Foresight (*3. Foresight Tuner*)

Foresight replaces all the PIDF and predictive-braking values from Pedro 2.x. Last season's numbers do **not** carry over.

- Inputs: **Distance** 48 in (max-velocity runs), **Velocity** 30 in/s (deceleration runs), **Braking distance** 36 in (at least 15).
- It runs these steps one after another. Each one drives the robot, so clear **4+ tiles in front, behind, left and right**:
  - Max forward velocity and max strafe velocity: drives 48 in at full power and drifts past.
  - Forward and strafe deceleration.
  - Heading braking and the heading tuner: spins in place.
  - Forward and strafe braking: drives back and forth at several speeds.
  - Forward and strafe translational: small moves of 12–24 in.
- If a step reports a failed calibration or a runaway, check the odometry again (step 1c) before retrying.
- At the end it prints `public static ForesightConfig foresightConfig = new ForesightConfig(c -> { ... });`. Paste it over `public static ForesightConfig foresightConfig = null;` in `pedro/Constants.java`.
- Rebuild, install, then run **4. Tests**:
  - **Hold Test:** push the robot. It should spring back to its spot without shaking.
  - **Line Test:** drives 48 in back and forth. It should end within about 1 in and not overshoot much.
  - **Curve Test / Interpolation Test:** follows a curve, turning while driving.

After this step the autos work. Until then, they stop at init with "Foresight is not tuned yet".

---

## 3. Flywheel PIDF (*TUNE: Flywheel PIDF*)

The robot doesn't drive in this OpMode. Controls:

| Button | Does |
|---|---|
| A | spin on / off |
| D-pad up / down | target ±50 ticks/s |
| D-pad left / right | select kF / kP / kI / kD |
| RB / LB | selected gain + / − (hold right trigger for 10× steps) |
| Y | selected gain = 0 |
| B (hold) | feed a ball (gate + intake), to see the speed dip |

Telemetry shows the target, actual speed, error, power, **rise time**, **overshoot**, **average error**, and the **lowest speed while feeding**.

1. **Start clean:** select kP, kI and kD and press Y on each, so only kF is left.
2. **kF:** a good starting value is `1 / Vmax` (from step 0, e.g. 1 / 2600 ≈ 0.00038). Set the target to 1300 and press A. Adjust kF until the actual speed settles **just under** the target (within about 30–50).
   - Check 1000 and 1800 as well: one kF should be close at every speed.
3. **kP:** raise it until the rise time gets short (under about 1 s) **without** the speed bouncing above and below the target. If it oscillates, back off by about 30%.
4. **Feed test:** hold B to shoot. Watch "lowest while feeding" and how fast it recovers. If it drops a lot and recovers slowly, raise kP a little.
5. **kD:** only if there's still overshoot or bounce. Small steps.
6. **kI:** usually 0. Use a tiny kI only if the speed always settles a bit below the target. It only builds up within 150 ticks/s of the target (`FlywheelPIDF.I_ZONE`).
7. Copy the line at the bottom of telemetry (`KF = …; KP = …; KI = …; KD = …;`) into `subsystems/FlywheelPIDF.java`.

The real Shooter uses this exact controller, so the match TeleOp and autos behave the same as the tuner.

---

## 4. Shooting LUT (*TUNE: LUT Finder (start 72,72)*)

The LUT maps **distance from the turret to the target** to **flywheel speed** and **hood position**.

Everything works like TeleOp (drive, turret auto-aim, LB to shoot, RT for intake), but speed and hood are manual:

| Button | Does |
|---|---|
| D-pad up / down | flywheel speed ±25 |
| D-pad left / right | hood ∓ 0.02 |
| B | load the current LUT value for this distance (starting point) |
| A | **record** distance + speed + hood |
| X | delete the last recorded point |

**The robot thinks it starts at (72, 72, 90°).** That's the center of the field, under the HIVE frame, so on a real field the robot can't be there. Change `LutFinder.START_X / START_Y / START_HEADING_DEG` to a real spot you can measure (for example a tile corner), and place the robot exactly there before pressing START. Triangle resets to that same spot.

If the robot isn't where it thinks it is, every distance is wrong and the LUT will be wrong.

Steps:
1. Choose the alliance in init (d-pad left for red, right for blue). The robot aims at that alliance's hive cell, picked by which half of the field it's in.
2. Drive to about 40 in from the target (telemetry shows **Distance**). **Stop moving.** Shooting while moving shifts the aim on purpose.
3. Press B to load the current LUT guess, then LB to shoot.
4. Adjust: if shots go **short**, raise the speed (or the hood); if **long**, lower it. Keep going until 3 shots in a row score.
5. Press **A** to record.
6. Move about 10 in farther and repeat, out to the longest shot you'll take (about 170 in).
7. Telemetry (and the robot log, tag `LUT`) prints ready-to-paste lines:
   ```
   LUT_DIST = { 40, 50, ... };
   LUT_VEL  = { 1050, 1100, ... };
   LUT_HOOD = { 0.45, 0.50, ... };
   ```
   Paste them into `subsystems/Shooter.java`, keeping `public static double[]` in front of each.
8. If the hood changes with distance, set `Shooter.USE_HOOD_LUT = true`. While it's false, the hood stays at `HOOD_POS` (0.5).

`TOF_DIST` / `LUT_TOF` (time of flight, for shoot-on-the-move) are separate and can stay as they are.

---

## 5. Check everything

- **TEST: TeleOp (start 72,72):** same as TeleOp, but always starts at (72, 72, 90°), never loads or saves the auto pose, and shows a **POSITION** block. Use it for driving practice and odometry checks without touching the real TeleOp's saved pose.
- **RED / BLUE AUTO 1 and 2:** the turret must face straight forward at INIT. Watch the "Step" line in telemetry.
- **TeleOp** after an auto: it should start where the auto ended, with the alliance already set.
