# Super README

This document is a code-level map of the robot software. It focuses on three things:

1. Where each subsystem gets its data from
2. Which commands exist and what they do
3. How the full shooting pipeline works from state estimation to feeding a ball

This file is meant to be a practical navigation document for future debugging and tuning.

## High-Level Architecture

The project is organized around command-based robot code with a few important layers:

- `RobotContainer`: constructs hardware abstractions, wires defaults, button bindings, and periodic global state updates
- `RobotStateRecorder`: shared pose, velocity, transform, and shot-frame state
- subsystem classes: mechanism-level control such as `TurretSubsystem`, `SpindexerSubsystem`, `IntakerSubsystem`, and the generic `VelocityMotorSubsystem` / `PositionMotorSubsystem`
- composition layer: `ShootingSuperstructure` combines turret, hood, shooter, and indexer into one shooting behavior
- calculators: `ShotCalculator` turns field state and robot motion into a desired shot solution
- configs and NT params: default constants live in `frc/robot/subsystems/Configs`, with runtime tuning exposed through generated `*ParamsNT` classes

## Runtime Data Sources

These are the main places the robot gets information from.

### Motor and mechanism feedback

Most mechanisms use:

- `MotorIOTalonFX` on real robot
- `MotorIOSim` in simulation
- `MotorInputsAutoLogged` for AdvantageKit logging

These provide:

- position
- velocity
- applied voltage
- stator current
- supply current

### Absolute encoders

The turret uses two absolute encoders:

- `CANCoderIOCANCoder` on real robot
- `CANCoderIOSim` in simulation

These are used to reconstruct the turret's absolute unwrapped angle on startup.

### Gyro / IMU

Swerve gyro data comes from:

- `lib/ironpulse/swerve/mk5n/ImuIOPigeon`

Important signals include:

- yaw position
- yaw velocity
- pitch position
- pitch velocity
- roll position
- roll velocity

The current robot yaw rate used by `RobotStateRecorder.getOmegaRobotCurrent()` is now sourced from this IMU chain through `Swerve.getYawVelocityRadPerSec()`.

### Vision

Vision is handled by:

- `lib/ironpulse/limelight/LimelightSubsystem`
- `lib/ironpulse/limelight/LimelightIOReal`

It feeds pose information into swerve pose estimation and also updates the field display.

### Field and shot geometry

Global robot and shot geometry are stored in:

- `frc/robot/RobotStateRecorder.java`

This is where the code stores:

- world-to-robot transform
- robot-to-shot transform
- robot velocity
- commanded robot velocity
- robot angular velocity
- current shot frame
- commanded shot frame

### Shot model files

The shot calculator loads table data from deploy JSON files:

- `results_GOAL.json`
- `results_PASS.json`

These are used to interpolate shot solutions by distance and robot motion.

### Runtime tunables

Many parameters come from generated NetworkTables-backed wrappers such as:

- `TurretVelParamsNT`
- `TurretPosParamsNT`
- `ShooterParamsNT`
- `SpindexerParamsNT`
- `SpindexerModeParamsNT`
- `ShotCalculatorParamsNT`
- `HoodParamsNT`
- `IntakerRollerParamsNT`
- `IntakerExtensionParamsNT`

Their source definitions live in `frc/robot/subsystems/Configs/*Config.java`.

## Subsystem Map

This section answers two questions for each subsystem:

- what it is
- where its data comes from

### Swerve

Primary class:

- `lib/ironpulse/swerve/Swerve`

Built in:

- `RobotContainer.buildSwerve()`

Real hardware sources:

- `ImuIOPigeon` for gyro signals
- `SwerveModuleIOMK5N` for each wheel module

Main responsibilities:

- read wheel and gyro state
- estimate robot pose
- expose measured and commanded chassis speeds
- accept joystick and auto drive commands

Important data outputs:

- `getEstimatedPose()`
- `getChassisSpeeds()`
- `getChassisSpeedsCmd()`
- `getYawVelocityRadPerSec()`

Default command:

- `SwerveCommands.driveWithJoystick(...)`

### Limelight

Primary class:

- `lib/ironpulse/limelight/LimelightSubsystem`

Built in:

- `RobotContainer.buildLimelight()`

Data comes from:

- real Limelight devices through `LimelightIOReal`
- robot heading and angular velocity suppliers from robot state / swerve

Main responsibilities:

- provide vision pose updates
- help field visualization
- reseed or align IMU-related data when requested

No dedicated default command is assigned in `RobotContainer`.

### Turret

Primary class:

- `frc/robot/subsystems/ShootingSubsystem/TurretSubsystem`

Built in:

- `RobotContainer.buildTurret()`

Data comes from:

- TalonFX mechanism feedback through `MotorIOTalonFX`
- two CANcoders for absolute startup reconstruction
- robot pose from `RobotStateRecorder`
- robot angular velocity compensation from `RobotStateRecorder.getOmegaRobotCurrent()` and world-velocity signals

Main responsibilities:

- convert world target angle into robot-relative target angle
- unwrap the target to avoid unnecessary rotation
- avoid turret soft limits
- switch between seeking and tracking behavior
- output velocity-based turret control

Default command:

- `turret.runTurretTargetLoop()`

### Hood

Implemented as:

- `PositionMotorSubsystem<MotorInputsAutoLogged, MotorIO, Angle>`

Built in:

- `RobotContainer.buildHood()`

Data comes from:

- TalonFX position feedback
- `RobotStateRecorder.getCmdFrame().hoodAngle()`

Main responsibilities:

- position the hood/backplate angle for the requested shot
- zero on enable

Default command:

- follow the commanded hood angle from `RobotStateRecorder.getCmdFrame()`

### Shooter

Implemented as:

- `VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO>`

Built in:

- `RobotContainer.buildShooter()`

Data comes from:

- TalonFX flywheel feedback
- setpoints generated by `ShootingSuperstructure.computeRpm(...)`

Main responsibilities:

- hold idle speed when not shooting
- track desired flywheel speed during shooting

Default command:

- idle at `ShooterParamsNT.idleVelRPS`

### Spindexer / Indexer

Primary class:

- `frc/robot/subsystems/ShootingSubsystem/SpindexerSubsystem`

Built in:

- `RobotContainer.buildSpindexer()`

Data comes from:

- TalonFX velocity and current feedback
- requested `IdxMode` from shooting logic

Main responsibilities:

- feed balls into the shooter
- reverse to unjam
- detect jams using filtered current plus low velocity

Default command:

- `idx.runState(() -> IdxMode.OFF)`

### Intake

Primary class:

- `frc/robot/subsystems/IntakerSubsystem`

Built from:

- roller as `VelocityMotorSubsystem`
- extension as `PositionMotorSubsystem`

Data comes from:

- roller TalonFX feedback
- extension TalonFX feedback
- extension current-based zeroing logic
- internal `currentMode` state

Main responsibilities:

- deploy or retract intake extension
- run intake roller
- feed toward the shooter
- run reverse while extended
- zero the extension

Default commands:

- roller default follows `currentMode`
- extension default follows `currentMode`

### Climber

Implemented as:

- `PositionMotorSubsystem<MotorInputsAutoLogged, MotorIO, Distance>`

Built in:

- `RobotContainer.buildClimber()`

Data comes from:

- climber motor feedback

Main responsibilities:

- mechanism position control for climb

Current note:

- `HAS_CLIMBER_IO` is `false`, so this is currently not running with real hardware IO from `RobotContainer`

Default command:

- `climber.runStop()`

### Indicator / LEDs

Primary class:

- `lib/ironpulse/indicator/IndicatorSubsystem`

Built in:

- `RobotContainer.buildIndicator()`

Data comes from:

- DriverStation mode
- alliance color
- `shootingSuperstructure.isShooting()`
- intake mode

Main responsibilities:

- show robot state on LEDs
- flash status after shooting
- indicate special hold conditions

Default command:

- pattern selection logic in `RobotContainer`

### ShootingSuperstructure

Primary class:

- `frc/robot/subsystems/ShootingSubsystem/ShootingSuperstructure`

This is not just a single hardware subsystem. It is the composition layer that owns:

- turret
- hood
- shooter
- spindexer
- `BallCounter`

Data comes from:

- `RobotStateRecorder.getCmdFrame()`
- `RobotStateRecorder.getPoseWorldShotCurrent()`
- shooter measured speed
- hood measured position
- turret mode
- field position for tower exclusion

Main responsibilities:

- run the full shooting set of mechanisms together
- compute flywheel RPM from desired muzzle speed
- gate indexer feed
- expose helper commands like `shootWhenReady()`

## Command Map

This section is organized by feature instead of by class so it is easier to use in pit debugging.

### Swerve-related commands

Main command:

- `SwerveCommands.driveWithJoystick(...)`
  - default teleop drive

Other swerve commands:

- `SwerveCommands.xLock(swerve)`
  - locks the modules in an X pattern
- `SwerveCommands.resetAngle(...)`
  - resets heading / odometry alignment

Bindings:

- driver sticks: default drive
- driver left stick: X-lock while shooting
- driver start: reset angle and reseed related systems

### Intake commands

From `IntakerSubsystem`:

- `runIntake()`
  - set intake mode to active intake
- `runExtendedIdle()`
  - extend and hold without normal feeding
- `runRetract()`
  - retract intake
- `toggleIntake()`
  - toggle between intake and extended idle fallback mode
- `runFeed()`
  - temporarily set intake mode to feeding
- `runExtendedReverse()`
  - temporarily reverse while extended
- `zeroCommand()`
  - normal extension zero command
- `outZeroCommand()`
  - current-based outer zeroing routine

Bindings:

- driver left bumper: toggle intake
- driver POV up: intake out-zero
- driver POV left: intake zero
- driver POV down: retract
- driver left trigger: feed
- operator left bumper: feed
- operator right bumper: extended reverse

### Hood commands

Used through the generic position subsystem plus superstructure:

- default hood tracking from `cmdFrame`
- `shootingSuperstructure.runZero()`
  - zero hood

Bindings:

- operator Y: zero hood
- enabled trigger: zero hood on enable

### Turret commands

From `TurretSubsystem`:

- `setTurretPoseWorld(...)`
  - update desired turret world angle supplier
- `runTurretTargetLoop()`
  - default turret tracking loop
- `runTurretToZero()`
  - preset move to zero angle

Direct operator bindings are not used. The turret is normally controlled through the superstructure.

### Shooter commands

Shooter mostly runs through the generic velocity subsystem:

- idle velocity default
- `runVelVolt(...)`
  - used by shooting logic to command RPM

Direct manual test commands exist in comments inside `RobotContainer`, but are not currently active bindings.

### Spindexer commands

From `SpindexerSubsystem`:

- `runState(...)`
  - choose OFF / FEED / REVERSE style behavior

From `ShootingSuperstructure`:

- `runUnjamming()`
  - reverse temporarily to clear jam
- `runForceFeeding()`
  - force feed regardless of normal automatic gating

Bindings:

- operator left trigger: unjam

### Superstructure shooting commands

From `ShootingSuperstructure`:

- `runFrame()`
  - command turret, hood, and shooter to follow `RobotStateRecorder.getCmdFrame()`
- `runFrame(Supplier<IdxMode>)`
  - same as above plus indexer mode
- `shootWhenReady(forceFeed)`
  - hold aim and spin-up, then feed only when ready conditions pass
- `shootWhenReady(forceFeed, GenericHID...)`
  - same plus rumble logic
- `shootOnCondition(...)`
  - conditional feed version often useful for auto
- `runSetFrame()`
  - fixed preset shot / feed frame
- `runResetBallCounter()`
  - reset shot counters

Bindings:

- operator A: `runSetFrame()`
- driver right trigger: `runSetFrame()`
- operator right trigger: `shootWhenReady(false, ...)`
- driver right bumper: `shootWhenReady(false, ...)`
- driver left stick: `xLock + shootWhenReady(false)`
- operator right stick: reset ball counter

### Auto-related command composition

Auto logic is centered around:

- `frc/robot/auto/AutoActions.java`
- `frc/robot/auto/AutoRoutines.java`
- `frc/robot/auto/AutoFile.java`

These compose:

- swerve path following
- intake states
- shooting superstructure commands
- spindexer / shooter support

`Robot.getAutonomousCommand()` returns `AutoFile.buildAuto()`.

## RobotContainer Responsibilities

`RobotContainer` is the central wiring file.

Main jobs:

- build all real or sim subsystem IO
- load shot model JSONs
- initialize auto helpers
- install button bindings
- assign default commands
- push shared robot state into `RobotStateRecorder` every loop

Important construction flow:

1. build `swerve`
2. build `limelightSubsystem`
3. build shooting mechanisms: `turret`, `shooter`, `spindexer`, `hood`
4. build `intake`
5. build `climber`
6. build `indicatorSubsystem`
7. initialize `ShotCalculator`
8. initialize auto classes
9. configure bindings
10. assign default commands

## RobotStateRecorder Responsibilities

`RobotStateRecorder` is one of the most important files in the project because it is the shared state bridge between:

- localization
- shot calculation
- turret control
- superstructure telemetry

It stores:

- `TWorldRobot`
- `TRobotShot`
- target frame names
- measured and commanded robot velocity
- gyro yaw rate
- current shot frame
- commanded shot frame

Key public methods:

- `putVelocityRobot(...)`
- `putVelocityRobotCmd(...)`
- `putOmegaRobotCurrent(...)`
- `getVelocityRobotCurrent()`
- `getVelocityWorldRobotCurrent()`
- `getVelocityRobotCmdCurrent()`
- `getVelocityWorldRobotCmdCurrent()`
- `getOmegaRobotCurrent()`
- `getPoseWorldRobotCurrent()`
- `getPoseWorldShotCurrent()`
- `getPoseDriverRobotCurrent()`
- `getPoseWorldTargetCurrent(...)`
- `getTranslationShotToTargetCurrent(...)`

## Full Shooting Logic

This is the most important end-to-end behavior in the robot.

### Step 1: update robot state

Every loop, `RobotContainer.robotPeriodic()` writes fresh data into `RobotStateRecorder`:

- robot pose from swerve odometry
- shot pose from hood angle plus turret angle
- measured chassis speeds
- commanded chassis speeds
- gyro yaw velocity

This creates the shared state needed for shot solving.

### Step 2: compute the desired shot frame

Still in `robotPeriodic()`, the code runs:

- `RobotStateRecorder.setCmdFrame(shotCalculator.computeShotFrame())`

`ShotCalculator` decides:

- whether the robot should target `GOAL` or `FEED`
- which target frame to use
- current shot-to-target vector
- current robot motion relative to that target

It then:

1. looks up an initial shot in the JSON shot table
2. estimates lookahead based on delay and flight time
3. predicts where the robot will effectively shoot from
4. does another table lookup with predicted distance and velocity
5. applies tuning offsets
6. solves final turret world yaw with lateral compensation

The result is a `ShotFrame` containing:

- target turret world angle
- hood angle
- desired muzzle speed

### Step 3: superstructure turns shot frame into mechanism commands

`ShootingSuperstructure.runFrame()` reads `RobotStateRecorder.getCmdFrame()` and sends commands to:

- turret
- hood
- shooter

Specifically:

- turret gets `cmdFrame.turretAngleWorld()`
- hood gets `cmdFrame.hoodAngle()`
- shooter gets RPM computed from `cmdFrame.muzzleSpeed()` and hood angle

### Step 4: turret tracks aim

`TurretSubsystem` converts world target angle into robot-relative angle.

Then it:

- unwraps the target to choose a practical rotation path
- avoids soft limits
- uses `SEEKING` mode for larger moves
- switches to `TRACKING` mode when close
- applies chassis rotation compensation

Important note:

- the turret does not just use shortest path blindly
- it also adjusts the path to avoid crossing soft limits

### Step 5: hood positions launch angle

The hood follows the commanded angle from the shot frame.

This determines projectile elevation and also updates the shot transform stored in `RobotStateRecorder`.

### Step 6: shooter reaches required flywheel speed

The shot model outputs muzzle speed in physical units. That is converted to shooter RPM inside `ShootingSuperstructure.computeRpm(...)`.

This conversion uses:

- `rpmA`
- `rpmB`
- `rpmC`
- `distanceScaler`

Distance scaling is applied for longer shots.

### Step 7: only then does the indexer feed

`shootWhenReady(...)` runs `runFrame()` in parallel with a wait gate.

The indexer does not immediately feed. It waits until:

- shooter velocity is at goal

Then the feed state is allowed only when:

- turret mode is `TRACKING`
- robot is not in the tower exclusion zone

If those conditions are not met, indexer remains `OFF`.

### Step 8: ball detection and counting

`BallCounter` is updated during RPM computation.

It detects a shot using:

- a rising edge in shooter current
- shooter near speed
- spindexer in `FEED`

It tracks:

- all balls
- goal balls
- per-shift counts
- rolling balls-per-second

## Shooting Logic by File

If you want to follow the shooting stack in code order, read these files in this order:

1. `frc/robot/RobotContainer.java`
2. `frc/robot/RobotStateRecorder.java`
3. `frc/robot/subsystems/ShootingSubsystem/ShotCalculator.java`
4. `frc/robot/subsystems/ShootingSubsystem/ShotFrame.java`
5. `frc/robot/subsystems/ShootingSubsystem/ShootingSuperstructure.java`
6. `frc/robot/subsystems/ShootingSubsystem/TurretSubsystem.java`
7. `frc/robot/subsystems/ShootingSubsystem/SpindexerSubsystem.java`
8. `frc/robot/subsystems/ShootingSubsystem/BallCounter.java`
9. `frc/robot/subsystems/Configs/TurretConfig.java`
10. `frc/robot/subsystems/Configs/ShotCalculatorConfig.java`
11. `frc/robot/subsystems/Configs/ShooterConfig.java`
12. `frc/robot/subsystems/Configs/HoodConfig.java`
13. `frc/robot/subsystems/Configs/IdxConfig.java`

## Notes and Useful Conventions

### Real vs sim hardware selection

Many mechanisms are built as:

- real IO if `RobotBase.isReal()` and `HAS_*_IO` is true
- sim IO otherwise

This selection happens inside `RobotContainer`.

### Generated NT parameter classes

Files like `ShotCalculatorParamsNT` are generated wrappers around config classes marked with `@NTParameter`.

You should edit the source config files, not the generated files.

### Generic motor subsystems

Many mechanisms are thin wrappers around the shared generic classes:

- `VelocityMotorSubsystem`
- `PositionMotorSubsystem`

That means the true "logic" for a mechanism may be split between:

- the config class
- the subsystem wrapper
- the superstructure or command that uses it

## Suggested Reading Order for New Developers

If someone is new to this codebase, this is the fastest useful order:

1. `RobotContainer`
2. `RobotStateRecorder`
3. `ShootingSuperstructure`
4. `ShotCalculator`
5. `TurretSubsystem`
6. `SpindexerSubsystem`
7. `IntakerSubsystem`
8. configs under `frc/robot/subsystems/Configs`
9. auto files under `frc/robot/auto`

## Quick Command Cheat Sheet

- drive: `SwerveCommands.driveWithJoystick`
- lock swerve: `SwerveCommands.xLock`
- intake toggle: `IntakerSubsystem.toggleIntake`
- intake feed: `IntakerSubsystem.runFeed`
- intake reverse: `IntakerSubsystem.runExtendedReverse`
- intake zero: `IntakerSubsystem.zeroCommand` / `outZeroCommand`
- hood zero: `ShootingSuperstructure.runZero`
- preset shot: `ShootingSuperstructure.runSetFrame`
- shoot automatically: `ShootingSuperstructure.shootWhenReady`
- conditional auto shoot: `ShootingSuperstructure.shootOnCondition`
- unjam indexer: `ShootingSuperstructure.runUnjamming`
- reset shot counters: `ShootingSuperstructure.runResetBallCounter`

## Final Summary

The robot's software revolves around a shared state pipeline:

- swerve and sensors estimate where the robot is
- `RobotStateRecorder` stores that state in a common frame graph
- `ShotCalculator` uses that state to compute a desired shot
- `ShootingSuperstructure` turns that desired shot into mechanism commands
- turret, hood, shooter, and spindexer execute the shot with safety and readiness gates

If you are debugging a shot problem, the fastest path is usually:

1. verify `RobotStateRecorder` pose and velocity
2. verify `ShotCalculator` output frame
3. verify turret mode and target path
4. verify hood angle and shooter RPM
5. verify indexer gating and tower exclusion
