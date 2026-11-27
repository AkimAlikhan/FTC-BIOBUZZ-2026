# Pedro Pathing 3.0: team notes (2026-27 BIOBUZZ)

Stack: FTC SDK **12.0**, `com.pedropathing:revhub:3.0.2`, `com.pedropathing:tuning:1.0.3` (AutoTune).
Verified against the 3.0.2 jars and the docs at <https://pedropathing.com/docs/pathing>.

Last season's code (Pedro 2.1.2, DECODE) is saved in `archive/2025-26-decode/`. It is reference only and is **not** compiled.

## Project layout

| File | What it is |
|---|---|
| `teamcode/pedro/Constants.java` | Drivetrain, localizer and Foresight configs, plus `create(hardwareMap)` → `Follower` |
| `teamcode/pedro/Tuning.java` | `@Tuner` procedures that appear on the AutoTune web page |
| `teamcode/pedro/procedures/` | Quickstart tuners (left unchanged so upstream updates merge cleanly) |
| `teamcode/opmodes/RedAuto1.java` | "RED AUTO 1": 18-step auto (paths + shots), saves end pose for TeleOp |
| `teamcode/opmodes/BlueAuto1.java` | "BLUE AUTO 1": same route rotated 180° around the field center; edit independently |
| `teamcode/opmodes/RedAuto2.java` / `BlueAuto2.java` | "RED/BLUE AUTO 2": shoot preload, one path, stop |
| `teamcode/opmodes/MainTeleOp.java` | "TeleOp": field-centric drive, alliance select, shoot on LB, intake on RT |
| `teamcode/opmodes/FlywheelEncoderTest.java` | Prints flywheel `getVelocity()` |
| `teamcode/opmodes/ServoTuner.java` | Find gate / hood servo positions |
| `teamcode/subsystems/Shooter.java` | Flywheel, intake, gate, hood, turret aim, shot sequence (shared by Auto + TeleOp) |
| `teamcode/subsystems/Field.java` | Red→blue mirroring and the Auto→TeleOp pose handoff (blackboard) |

## Driver Hub robot configuration names

| Name | Type | Notes |
|---|---|---|
| `fl` | Motor | front left drive |
| `fr` | Motor | front right drive |
| `bl` | Motor | back left drive |
| `br` | Motor | back right drive |
| `flywheel` | Motor | the single flywheel motor (encoder plugged in, REVERSE in code) |
| `intakeLeft` | Motor | intake roller 1 |
| `intakeRight` | Motor | intake roller 2 (same direction as intakeLeft) |
| `bearing` | Motor | turret |
| `pinpoint` | I2C: goBILDA Pinpoint | odometry computer |
| `turnAngle` | Servo | hood (single servo) |
| `gate` | Servo | optional; TeleOp skips it if missing |

## Bring-up order (AutoTune)

1. Install the app, connect to the robot Wi-Fi, and open the AutoTune page.
2. **1. Mecanum Tuner**: motor names and directions → paste over `drivetrainConfig`.
3. **2. Pinpoint Tuner**: pod offsets and directions → paste over `localizerConfig`.
4. **4. Tests** → Localization / Odometry / Driving / Pose. These work before Foresight is tuned. Do not tune Foresight on bad localization.
5. **3. Foresight Tuner** → paste over `foresightConfig` (replace `null`).
6. **4. Tests** → Hold / Line / Curve. After this, `Constants.create()` works and the Autos and TeleOp run.

`Constants.create()` throws a clear error until step 5 is done. `ForesightConfig` rejects a config that is missing its tuned values, so there are no placeholder numbers.

## 2.x → 3.0 cheat sheet (what our old code used)

| Pedro 2.x | Pedro 3.0 |
|---|---|
| `FollowerConstants`, `MecanumConstants`, `PinpointConstants`, `FollowerBuilder` | `new Follower(localizer, drivetrain, algorithm)`, configured as `new MecanumConfig(c -> {...})` / `PinpointConfig` / `ForesightConfig` |
| Translational, heading and drive PIDF, predictive braking, zero-power accel | **Foresight**. All of it comes from the Foresight Tuner. Old numbers do not carry over. |
| `follower.pathBuilder().addPath(new BezierLine(a, b)).setLinearHeadingInterpolation(..).build()` | `line(a, b).linear(a, b)`, with `import static com.pedropathing.api.Paths.*` |
| `new BezierCurve(a, c1, b)` | `curve(a, c1, b)`, where the first pose is the start and the last is the end |
| `addPath(...).addPath(...)` chains | `path(p1, p2, ...)` concatenates `Path`s |
| `setTangentHeadingInterpolation()` | `.tangent()`, plus `.reverseTangent()` |
| `setConstantHeadingInterpolation(h)` | `.constant(h)` or `.constant(pose)` |
| (none) | `.facingPoint(pose)`: heading tracks a target point |
| `follower.followPath(chain)` | `follower.follow(path)` |
| `follower.isBusy()` | `follower.isBusy()` (unchanged), plus `atParametricEnd()` as the looser check |
| `follower.setMaxPower(x)` | Per path: `.with(Constants.foresightConfig.maxPathSpeed.at(0.7))` (fraction of full speed) |
| `follower.getPose()` / `setStartingPose()` | `follower.pose()` / `follower.setPose()` |
| `startTeleopDrive()` + `setTeleOpDrive(...)` | `follower.manual(fwd, strafe, turn)`, or `ManualDrive.driveOrHold(follower, powers)` |
| Hand-written field-centric | `ManualDrive.fieldCentric(fwd, strafe, turn, heading)` |
| Path callbacks | Gone. Use a state machine (see `RedAuto1`) or the Ivy command library. |
| `new Pose(x, y, Math.toRadians(h))` | Still works. `PoseFactory.degrees().of(x, y, h)` is nicer, and `.mirrorX(72)` gives the other alliance. |

### Conventions (checked in the library code)
- Units: inches, and radians for heading. `PoseFactory.degrees()` converts for you.
- Robot frame: **+forward, +strafe = LEFT, +turn = counter-clockwise.** Negate `left_stick_x` and `right_stick_x`. The docs' TeleOp example leaves them un-negated, which drives mirrored.
- `follower.update()` must run every loop, in Auto and TeleOp.
- `follower.mode()` is one of `FOLLOW`, `HOLD`, `MANUAL`, `IDLE`. After a path ends the follower holds the endpoint (`holdEnd`).
- `PoseFactory.mirrorX(72)`: x → 144 − x, heading → π − heading.
- Any `ConfigVar` gives a per-path override: `config.someVar.at(value)` returns a `Modifier`, which you pass to `path.with(...)`.
