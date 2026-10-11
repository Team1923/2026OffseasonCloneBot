# Simulation, logging, and replay

This guide covers running the robot code in the WPILib Simulation GUI, choosing
**simulate** or **replay** from the dashboard, watching data in AdvantageScope,
and collecting logs from the real robot. No code changes are needed to switch
between any of these.

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
4. The VS Code terminal prints:

   ```text
   [Robot Startup] Choose options under "Robot Startup" and press Start
   ```

   **The robot code is paused until you press Start** (section 3). This is
   expected.

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

## 3. Choose what to run (Robot Startup)

The code publishes a **Robot Startup** folder under **SmartDashboard**:

| Entry | Choices | Meaning |
|---|---|---|
| **Run Mode** | `Simulate` | Normal simulation you can drive. |
| | `Replay (as fast as possible)` | Re-runs a log in a few seconds and writes a new `_sim` log. |
| | `Replay (real time, watch live)` | Re-runs a log at normal speed so you can watch it in AdvantageScope. |
| **Replay Source** | `Newest log in logs folder` | Newest `.wpilog` in this project's `logs/` folder (skips `_sim` outputs). |
| | `Log open in AdvantageScope` | Whatever log is currently open in AdvantageScope. |
| | `Replay Log Path field` | The file typed into **Replay Log Path**. |
| **Log Output** | `Default (...)` | Simulate → `logs/`. Replay → next to the replayed log, named `<name>_sim.wpilog`. |
| | `Logs folder` | Always the project's `logs/` folder. |
| | `Log Folder field` | The folder typed into **Log Folder**. |
| | `Don't save a log` | Nothing is written (live viewing still works). |
| **Replay Log Path** | text | Full path to a `.wpilog`, e.g. `C:\Users\you\Downloads\akit_26-03-14_10-02-11.wpilog`. |
| **Log Folder** | text | Folder for output logs, e.g. `logs\replays`. |
| **Start** | true/false | Set to **true** to begin. It resets to false by itself. |
| **Status** | read-only | What's happening, or why it can't start. |

Your choices are **remembered** for the next launch, so after the first time you
usually only need to press **Start**. Only choices that start successfully are
saved. If something is wrong (no log found, empty folder field), **Status**
explains why and the code keeps waiting so you can fix it and press Start again.

Replay Source, Replay Log Path, and Log Folder are ignored when they don't apply
(for example, Replay Source is ignored in `Simulate`).

You can set these from the Sim GUI itself, or from a dashboard app. Use whichever
you prefer.

### Option A: from the Sim GUI

1. In the **NetworkTables** window, expand **Transitory Values** →
   **SmartDashboard** → **Robot Startup**.
2. For the three dropdowns, open the menu **NetworkTables → SmartDashboard** and
   pick **Robot Startup/Run Mode** (then **Replay Source** and **Log Output**).
   Each opens a small window with a dropdown.
   - If you prefer the tree: expand **Run Mode** and edit its **selected** value
     to exactly one of the choice names from the table above.
3. For **Replay Log Path** / **Log Folder**, click the value in the tree and type.
4. Set **Start** to **true** (checkbox in the tree).
5. Watch **Status** and the VS Code terminal. You should see
   `[Robot Startup] Running: Simulate` (or the replay message).

### Option B: from Elastic (easier to click)

1. `Ctrl+Shift+P` → **WPILib: Start Tool** → **Elastic**.
2. In Elastic's settings, set the connection (IP address mode) to **Localhost** so
   it connects to the simulator.
3. Add widgets for the `SmartDashboard/Robot Startup` entries: the choosers show
   up as dropdowns, **Start** as a toggle, and **Status** as text. Save the layout
   so it's there next time.
4. Pick your options and toggle **Start**.

Shuffleboard also works: the entries appear on its **SmartDashboard** tab. Right-click
**Start** → *Show as* → **Toggle Button** so you can click it.

---

## 4. Simulate and drive

### 4.1 Start it
In Robot Startup choose **Run Mode = Simulate** and set **Start** to true.
The terminal shows `[AdvantageKit] Logging to "logs\akit_....wpilog"` (unless
Log Output is *Don't save a log*).

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
- **A log from this computer's sim runs**: it's already in `logs/`. Use
  *Newest log in logs folder*.
- **A log from the robot**: download it (section 6), then either
  - save it into this project's `logs/` folder and use *Newest log in logs folder*, or
  - open it in AdvantageScope (**File → Open Log(s)…**) and use
    *Log open in AdvantageScope* (keep AdvantageScope open), or
  - paste its full path into **Replay Log Path** and use *Replay Log Path field*.

### 5.2 Fast replay (check results)
1. Start the simulator (section 2).
2. Robot Startup: **Run Mode = Replay (as fast as possible)**, choose the
   **Replay Source**, leave **Log Output = Default**, set **Start** to true.
3. The terminal shows:

   ```text
   [Robot Startup] Replaying in a new process without the Sim GUI (AdvantageKit requires this). ...
   HAL Extensions: No extensions found
   [AdvantageKit] Logging to "logs\akit_..._sim.wpilog"
   [AdvantageKit] Log sent to AdvantageScope.
   ```

4. It finishes in seconds and exits. The **Robot Simulation** window closes by
   itself. If AdvantageScope is open, the `_sim` log opens automatically.

### 5.3 Real-time replay (watch it happen)
1. Same as above but **Run Mode = Replay (real time, watch live)**.
2. In AdvantageScope, **File → Connect to Simulator** (reconnect if it was already
   connected; the replay runs in a new process).
3. The replay runs at match speed and exits when the log ends.

### 5.4 Why the Sim GUI "freezes" during replay
AdvantageKit refuses to replay while the Sim GUI is loaded, because the GUI's
Driver Station controls would overwrite the Driver Station data in the log. So
when you press Start on a replay, the code starts a second copy of itself
**without** the Sim GUI and hands it the replay settings. The Robot Simulation
window stays open but does nothing, and it closes when the replay ends.
**Closing that window stops the replay.**

### 5.5 Compare the original and the replay
Open both logs in AdvantageScope (**File → Open Log(s)…**, select both files).
- Inputs (`Swerve/...`, `Intake/...`, ...) are identical. They came from the log.
- Outputs (`RealOutputs/...`) in the `_sim` log are what your current code
  decided. Differences show the effect of your code change.

### 5.6 Limitation
The drivetrain pose is computed inside CTRE's swerve library on its own thread, so
during replay `Swerve/Pose` comes straight from the log. Changes to odometry or
vision fusion can't be tested with replay; everything built on top of the pose
(aiming, state machines, commands) can.

### 5.7 Replay repeatedly while editing code
Run `./gradlew replayWatch` in the terminal and open the log in AdvantageScope.
Every time you save a file, it rebuilds and replays automatically. This skips the
Robot Startup selector.

---

## 6. Logging on the real robot

1. Plug a **USB stick** into the roboRIO (FAT32 is the safest format). Logs are
   written to `/U/logs`. **Without a USB stick nothing is saved.**
2. Deploy code as usual. On the robot there is no Robot Startup selector; it starts
   logging immediately.
3. A new `.wpilog` is created every time the robot code starts. During an FMS match
   the file name includes the event and match number.
4. **Watch live**: AdvantageScope → **File → Connect to Robot** (set the team number
   in AdvantageScope's preferences first).
5. **Get the logs**: AdvantageScope → **File → Download Logs…** while connected to
   the robot, or remove the USB stick and copy the files. Save them into this
   project's `logs/` folder if you want *Newest log in logs folder* to find them.

---

## 7. Troubleshooting

| Problem | Fix |
|---|---|
| Robot doesn't move in sim | Did you press **Start**? (terminal still says "Choose options…"). Is **Robot State** on **Teleoperated**? Is the controller in **Joystick[0]**? |
| Can't find the Robot Simulation window | Check the taskbar; it may be minimized. |
| Status: `no .wpilog files in logs` | Run a simulation first, or put a log in `logs/`, or use another Replay Source. |
| Status: `no log is open in AdvantageScope` | Open the log in AdvantageScope (**File → Open Log(s)…**) and keep it running. |
| Status: `replay log not found: ...` | Check the path in **Replay Log Path**; it must be a `.wpilog` file. |
| Status: `Log Folder is empty` | Type a folder into **Log Folder**, or choose a different Log Output. |
| Red error `Failed to load PathPlanner config` at startup | Expected for now: the robot config hasn't been created in the PathPlanner app yet. Driving still works; autos don't. |
| Want to reset the remembered choices | Close the sim and delete `networktables.json` in the project folder. |
| VS Code shows errors for classes that exist | Close old editor tabs without saving, then `Ctrl+Shift+P` → **Java: Clean Java Language Server Workspace**. |
| AdvantageScope shows no data in real-time replay | Reconnect: **File → Connect to Simulator** after pressing Start. |
| New sim won't start / port in use | A previous sim is still running. Close its window or stop it in Task Manager (`java.exe`). |
