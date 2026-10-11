"""What the analyzer knows about this robot: which log keys belong to which mechanism, the
tolerances and limits the code uses, and which source files to open when something is wrong.

KEEP THIS IN SYNC WITH THE CODE. When a key, tolerance, current limit, soft limit, or file
moves, update it here in the same change. Values below match Constants.java on the
gear-ratios-soft-limits branch (hood 0-37 deg, intake arm 0-120 deg).

Key spelling: no leading slash. Inputs (sensor values) live at the root, e.g. "Intake/...".
Outputs (code decisions) are written as "RealOutputs/..."; the analyzer automatically falls
back to "ReplayOutputs/..." for replayed logs.
"""

SRC = "src/main/java/frc/robot"

MECHANISMS = {
    "intake_arm": {
        "label": "Intake arm (pivot)",
        "kind": "position",
        "units": "deg",
        "measured": "Intake/ArmPositionDegrees",
        "velocity": "Intake/ArmVelocityDegreesPerSecond",
        "setpoint": "RealOutputs/Intake/Arm/SetpointDegrees",
        "applied_state": "RealOutputs/Intake/Arm/AppliedState",
        "desired_state": "RealOutputs/StateHandler/Intake/DesiredArmState",
        "current_state": "RealOutputs/StateHandler/Intake/CurrentArmState",
        "volts": "Intake/ArmAppliedVolts",
        "stator": "Intake/ArmStatorCurrentAmps",
        "supply": "Intake/ArmSupplyCurrentAmps",
        "temp": "Intake/ArmTemperatureCelsius",
        "connected": "Intake/ArmConnected",
        "tolerance": 3.0,  # IntakeConstants.ARM_POSITION_TOLERANCE_DEGREES
        "stator_limit": 90.0,
        "supply_limit": 40.0,
        "soft_limits": (0.0, 120.0),
        "motors": "1x Kraken X60, 20:1",
        "files": [
            f"{SRC}/subsystems/intake/IntakeSubsystem.java",
            f"{SRC}/subsystems/intake/IntakeIO.java",
            f"{SRC}/statemachines/IntakeStateMachine.java",
            f"{SRC}/Constants.java (IntakeConstants.ARM_CONFIG)",
        ],
    },
    "intake_roller": {
        "label": "Intake rollers",
        "kind": "voltage",
        "units": "V",
        "measured": "Intake/RollerAppliedVolts",
        "velocity": "Intake/RollerVelocityRpm",
        "setpoint": "RealOutputs/Intake/Roller/SetpointVolts",
        "applied_state": "RealOutputs/Intake/Roller/AppliedState",
        "desired_state": "RealOutputs/StateHandler/Intake/DesiredRollerState",
        "current_state": "RealOutputs/StateHandler/Intake/CurrentRollerState",
        "volts": "Intake/RollerAppliedVolts",
        "stator": "Intake/RollerStatorCurrentAmps",
        "supply": "Intake/RollerSupplyCurrentAmps",
        "temp": "Intake/RollerTemperatureCelsius",
        "connected": "Intake/RollerConnected",
        "tolerance": 1.0,  # volts; open loop, so this only catches limiting/brownout
        "stator_limit": 90.0,
        "supply_limit": 40.0,
        "motors": "Kraken X60, 2:1",
        "files": [
            f"{SRC}/subsystems/intake/IntakeSubsystem.java",
            f"{SRC}/subsystems/intake/IntakeIO.java",
            f"{SRC}/statemachines/IntakeStateMachine.java",
            f"{SRC}/Constants.java (IntakeConstants.ROLLER_CONFIG)",
        ],
    },
    "floor_roller": {
        "label": "Hopper floor roller",
        "kind": "velocity",
        "units": "RPM",
        "measured": "Rollers/FloorVelocityRpm",
        "setpoint": "RealOutputs/Rollers/Floor/SetpointRpm",
        "applied_state": "RealOutputs/Rollers/Floor/AppliedState",
        "desired_state": "RealOutputs/StateHandler/Rollers/DesiredFloorState",
        "current_state": "RealOutputs/StateHandler/Rollers/CurrentFloorState",
        "volts": "Rollers/FloorAppliedVolts",
        "stator": "Rollers/FloorStatorCurrentAmps",
        "supply": "Rollers/FloorSupplyCurrentAmps",
        "temp": "Rollers/FloorTemperatureCelsius",
        "connected": "Rollers/FloorConnected",
        "tolerance": 100.0,  # RollersConstants.RPM_TOLERANCE
        "stator_limit": 60.0,
        "supply_limit": 40.0,
        "motors": "1x Kraken X60, 3:1",
        "files": [
            f"{SRC}/subsystems/rollers/RollersSubsystem.java",
            f"{SRC}/subsystems/rollers/RollersIO.java",
            f"{SRC}/statemachines/RollersStateMachine.java",
            f"{SRC}/Constants.java (RollersConstants.MOTOR_CONFIG)",
        ],
    },
    "hood": {
        "label": "Shooter hood",
        "kind": "position",
        "units": "deg",
        "measured": "Shooter/HoodPositionDegrees",
        "velocity": "Shooter/HoodVelocityDegreesPerSecond",
        "setpoint": "RealOutputs/Shooter/Hood/SetpointDegrees",
        "applied_state": "RealOutputs/Shooter/Hood/AppliedState",
        "desired_state": "RealOutputs/StateHandler/Shooter/DesiredHoodState",
        "current_state": "RealOutputs/StateHandler/Shooter/CurrentHoodState",
        "volts": "Shooter/HoodAppliedVolts",
        "stator": "Shooter/HoodStatorCurrentAmps",
        "supply": "Shooter/HoodSupplyCurrentAmps",
        "temp": "Shooter/HoodTemperatureCelsius",
        "connected": "Shooter/HoodConnected",
        "tolerance": 0.5,  # ShooterConstants.HOOD_TOLERANCE_DEGREES
        "stator_limit": 40.0,
        "supply_limit": 30.0,
        "soft_limits": (0.0, 37.0),
        "motors": "1x Kraken X44, 5:2",
        "files": [
            f"{SRC}/subsystems/shooter/ShooterSubsystem.java (homeHoodCommand, states)",
            f"{SRC}/subsystems/shooter/ShooterIO.java",
            f"{SRC}/statemachines/ShooterStateMachine.java",
            f"{SRC}/Constants.java (ShooterConstants.HOOD_CONFIG)",
        ],
    },
    "drum": {
        "label": "Shooter drum",
        "kind": "velocity",
        "units": "RPM",
        "measured": "Shooter/DrumVelocityRpm",
        "setpoint": "RealOutputs/Shooter/Drum/SetpointRpm",
        "applied_state": "RealOutputs/Shooter/Drum/AppliedState",
        "desired_state": "RealOutputs/StateHandler/Shooter/DesiredDrumState",
        "current_state": "RealOutputs/StateHandler/Shooter/CurrentDrumState",
        # Arrays, one value per motor: leader, follower one, two, three.
        "volts": "Shooter/DrumAppliedVolts",
        "stator": "Shooter/DrumStatorCurrentAmps",
        "supply": "Shooter/DrumSupplyCurrentAmps",
        "temp": "Shooter/DrumTemperatureCelsius",
        "connected": "Shooter/DrumConnected",
        "motor_names": ["leader", "follower one", "follower two", "follower three"],
        # Follower alignment set in ShooterIO.Real: +1 Aligned, -1 Opposed (relative to leader).
        "follower_alignment": [1, 1, -1, -1],
        "tolerance": 75.0,  # ShooterConstants.DRUM_TOLERANCE_RPM
        "stator_limit": 120.0,
        "supply_limit": 60.0,
        "motors": "4x Kraken X60 (leader + 3 followers), 13:9",
        "files": [
            f"{SRC}/subsystems/shooter/ShooterSubsystem.java",
            f"{SRC}/subsystems/shooter/ShooterIO.java (follower setup)",
            f"{SRC}/statemachines/ShooterStateMachine.java",
            f"{SRC}/Constants.java (ShooterConstants.DRUM_CONFIG, InterpolationConstants)",
        ],
    },
}

SWERVE = {
    "label": "Swerve drivetrain",
    "module_names": ["front left", "front right", "back left", "back right"],
    "drive_stator": "Swerve/DriveStatorCurrentAmps",
    "drive_supply": "Swerve/DriveSupplyCurrentAmps",
    "drive_temp": "Swerve/DriveTemperatureCelsius",
    "drive_connected": "Swerve/DriveConnected",
    "steer_stator": "Swerve/SteerStatorCurrentAmps",
    "steer_temp": "Swerve/SteerTemperatureCelsius",
    "steer_connected": "Swerve/SteerConnected",
    "encoder_connected": "Swerve/EncoderConnected",
    "gyro_connected": "Swerve/GyroConnected",
    "failed_daqs": "Swerve/OdometryFailedDaqs",
    "heading_error": "RealOutputs/Swerve/FacingHeading/ErrorDegrees",
    "heading_tolerance": 5.0,  # SwerveConstants.HEADING_TOLERANCE_DEGREES
    "files": [
        f"{SRC}/subsystems/swerve/SwerveSubsystem.java",
        f"{SRC}/subsystems/swerve/SwerveIO.java",
        f"{SRC}/statemachines/SwerveStateMachine.java",
        f"{SRC}/lib/swerve/TunerConstants.java (generated; IDs, offsets, gains)",
    ],
}

VISION_CAMERAS = ["FrontLeft", "FrontRight", "RearLeft", "RearRight"]

SHOOTING = {
    "ready": "RealOutputs/Shooter/ReadyToShoot",
    # Commands that mean "the driver is trying to shoot right now".
    "shoot_commands": ["RealOutputs/Commands/ShootGamePieces", "RealOutputs/Commands/FerryGamePieces"],
    "homed": "RealOutputs/Shooter/Hood/Homed",
}

# Robot-wide signals written by AdvantageKit itself. Fallbacks cover plain WPILib DataLogManager
# logs, which use "DS:enabled" style names.
DS_ENABLED = ["DriverStation/Enabled", "DS:enabled"]
DS_AUTO = ["DriverStation/Autonomous", "DS:autonomous"]
DS_TEST = ["DriverStation/Test", "DS:test"]
BATTERY = ["SystemStats/BatteryVoltage", "systemTime/BatteryVoltage"]
BROWNOUT = "SystemStats/BrownedOut"
COMMS_DISABLE = "SystemStats/CommsDisableCount"
CAN_UTIL = "SystemStats/CANBus/Utilization"
CAN_OFF = "SystemStats/CANBus/OffCount"
CAN_TX_FULL = "SystemStats/CANBus/TxFullCount"
CAN_RX_ERR = "SystemStats/CANBus/ReceiveErrorCount"
CAN_TX_ERR = "SystemStats/CANBus/TransmitErrorCount"
LOOP_TIME = "RealOutputs/LoggedRobot/FullCycleMS"
CONSOLE = "RealOutputs/Console"

# Thresholds used for flags in the report.
TEMP_WARN_C = 70.0
TEMP_CRIT_C = 90.0
BATTERY_WARN_V = 9.0
BATTERY_CRIT_V = 7.5
LOOP_OVERRUN_MS = 25.0
