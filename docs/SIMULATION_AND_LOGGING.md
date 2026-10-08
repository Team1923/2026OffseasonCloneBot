# Simulation, logging, and replay

This guide covers running the robot code in the WPILib Simulation GUI, replaying
logs from the command line, watching data in AdvantageScope, and collecting logs
from the real robot.

The robot uses [AdvantageKit](https://docs.advantagekit.org/). Every sensor
reading (inputs) and every decision the code makes (outputs) is written to a
`.wpilog` file. Replay feeds a log's inputs back through the current code, so you
can see what your *changed* code would have done in a real match.

---

## 1. One-time setup

1. Install **WPILib 2026** (it includes VS Code, AdvantageScope, and Elastic).
2. Open this project folder in the **WPILib VS Code**.
3. Build once so dependencies download: `Ctrl+Shift+P` → **WPILib: Build Robot Code**.
4. Optional but strongly recommended: plug an **Xbox controller** into the
   computer. The keyboard can't turn the robot without extra setup (see 4.3).

---

## 2. Start the simulator

1. In VS Code press `Ctrl+Shift+P` and run **WPILib: Simulate Robot Code**.
2. A popup asks which extensions to use. Check **Sim GUI** and click OK.
   Leave **DS Socket** unchecked unless you want to use the real FRC Driver
   Station app instead of the Sim GUI.
3. The code builds, and a window titled **Robot Simulation** opens. If you don't
   see it, look for it in the taskbar; it sometimes opens minimized or behind VS
   Code.
4. The robot code connects normally as soon as construction finishes. The terminal prints:

   ```text
   ********** Robot program startup complete **********
   ```

   AdvantageKit also starts writing a log under `logs/` automatically.

### The Sim GUI at a glance

The menu bar has **Workspace, View, Hardware, NetworkTables, DS, Plot**. The
default windows are:

| Window | What it's for |
|---|---|
| **Robot State** | Disconnected / Disabled / Autonomous / Teleoperated / Test. Click one to change mode. |
| **Timing** | Run / Pause / Step the simulated clock. |
| **System Joysticks** | Controllers plugged into your computer, plus Keyboard 0–3. |
| **Joysticks** | The six driver-station joystick slots the robot code reads. |
| **FMS** | Alliance station (Red 1, Blue 2, …), match time, game-specific message. |
| **NetworkTables** | Live NetworkTables values, including the SmartDashboard folder. |
| **NetworkTables Info** | Who is connected to the simulator (dashboards, AdvantageScope). |
| **Other Devices** | Simulated devices. |

If you close a window by accident, reopen it from the menu bar.

---

## 3. Startup behavior

Normal desktop launches now run simulation immediately. There is no Robot
Startup dashboard selector and no extra Start button. Use
`./gradlew replayWatch` for replay.

---

## 4. Simulate and drive

### 4.1 Start it
Starting **WPILib: Simulate Robot Code** is sufficient. The terminal shows
`[AdvantageKit] Logging to "logs\akit_....wpilog"`, and the robot code connects
to the simulation Driver Station automatically.

### 4.2 Connect a controller
1. In **System Joysticks**, find your controller (e.g. `0: Xbox Controller`).
2. **Drag it** onto **Joystick[0]** in the **Joysticks** window. Slot 0 is the
   driver, slot 1 is the operator.
3. If it's an Xbox controller, make sure the Joysticks window treats it as a
   gamepad (a **Map gamepad** checkbox appears under the slot) so the sticks map
   correctly.

### 4.3 Keyboard instead of a controller
Drag **Keyboard 0** onto **Joystick[0]**. By default Keyboard 0 only has three
axes (A/D, W/S, E/R). The code turns with the Xbox **right stick X (axis 4)**, which
the keyboard doesn't have, so you can drive but not rotate. To add more axes, open
**Keyboard 0 Settings** from the **DS** menu and increase the axis count, then
assign keys to axis 4.

### 4.4 Enable and drive
1. In **Robot State**, click **Teleoperated**.
2. **Left stick** drives (field-centric), **right stick left/right** rotates.
3. Click **Disabled** to stop.

What to expect right now:
- **Swerve drives** in simulation (CTRE's built-in module simulation).
- **Intake, rollers, and shooter** log data but don't move: no buttons are bound
  yet and their PID gains/setpoints are still 0.
- Vision contributes nothing in sim (Limelights can't be simulated).

### 4.5 Watch it in AdvantageScope
1. `Ctrl+Shift+P` → **WPILib: Start Tool** → **AdvantageScope**.
2. **File → Connect to Simulator**.
3. Useful views (drag fields from the left sidebar onto a tab; add tabs with **+**):
   - **2D/3D Field**: drag `Swerve/Pose` to see the robot move.
   - **Swerve**: drag `Swerve/ModuleStates` (and `Swerve/ModuleTargets`).
   - **Line Graph**: e.g. `Shooter/DrumVelocityRpm`, `Intake/ArmPositionDegrees`,
     `Swerve/DriveStatorCurrentAmps`.
   - `RealOutputs/StateHandler/...`: every desired/current mechanism state.
   - `RealOutputs/Commands/...`: which commands are running.
   - `RealOutputs/Alerts`: disconnected devices and other alerts.

### 4.6 Stop
Close the **Robot Simulation** window, or click the trash-can icon on the VS Code
terminal. The log is finished and renamed with the date and time, e.g.
`logs\akit_26-09-13_15-25-00.wpilog`.

---

## 5. Replay a log

Replay shows what the **current code** decides given the **inputs recorded** in a
log. Use it to test a code change against a real match or an earlier sim run.

### 5.1 Pick the log
- **A log from this computer's sim runs** is already in `logs/`.
- **A log from the robot** can be downloaded using the steps in section 6.

Open the log in AdvantageScope with **File → Open Log(s)…** and keep
AdvantageScope running.

### 5.2 Fast replay (check results)
Run `./gradlew replayWatch` in the project terminal. AdvantageKit replays the
log without the Sim GUI and writes the recomputed output beside the source log
with `_sim` in its filename.

### 5.3 Compare the original and the replay
Open both logs in AdvantageScope (**File → Open Log(s)…**, select both files).
- Inputs (`Swerve/...`, `Intake/...`, ...) are identical. They came from the log.
- Outputs (`RealOutputs/...`) in the `_sim` log are what your current code
  decided. Differences show the effect of your code change.

### 5.4 Limitation
The drivetrain pose is computed inside CTRE's swerve library on its own thread, so
during replay `Swerve/Pose` comes straight from the log. Changes to odometry or
vision fusion can't be tested with replay; everything built on top of the pose
(aiming, state machines, commands) can.

### 5.5 Replay repeatedly while editing code
Run `./gradlew replayWatch` in the terminal and open the log in AdvantageScope.
Every time you save a file, it rebuilds and replays automatically.

---

## 6. Logging on the real robot

1. Plug a **USB stick** into the roboRIO (FAT32 is the safest format). Logs are
   written to `/U/logs`. **Without a USB stick nothing is saved.**
2. Deploy code as usual; logging starts immediately.
3. A new `.wpilog` is created every time the robot code starts. During an FMS match
   the file name includes the event and match number.
4. **Watch live**: AdvantageScope → **File → Connect to Robot** (set the team number
   in AdvantageScope's preferences first).
5. **Get the logs**: AdvantageScope → **File → Download Logs…** while connected to
   the robot, or remove the USB stick and copy the files. Save them into this
   project's `logs/` folder for local analysis and replay.

---

## 7. Troubleshooting

| Problem | Fix |
|---|---|
| Robot doesn't move in sim | Is **Robot State** on **Teleoperated**? Is the controller in **Joystick[0]**? |
| Can't find the Robot Simulation window | Check the taskbar; it may be minimized. |
| VS Code shows errors for classes that exist | Close old editor tabs without saving, then `Ctrl+Shift+P` → **Java: Clean Java Language Server Workspace**. |
| New sim won't start / port in use | A previous sim is still running. Close its window or stop it in Task Manager (`java.exe`). |
