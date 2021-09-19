package frc.robot.driver;

public enum DigitalOperation implements IOperation
{
    PositionResetFieldOrientation,

    // Vision operations:
    VisionForceDisable,
    VisionDisableStream,
    VisionEnablePowercellProcessing,
    VisionEnableRetroreflectiveProcessing,

    // Compressor operations:
    CompressorForceDisable,

    // DriveTrain operations:
    DriveTrainPathMode,
    DriveTrainReset,
    DriveTrainEnableFieldOrientation,
    DriveTrainDisableFieldOrientation,
    DriveTrainEnableMaintainDirectionMode,
    DriveTrainDisableMaintainDirectionMode,

    // Intake operations:
    PowerCellIntakeExtend,
    PowerCellIntakeRetract,
    PowerCellIntake,
    PowerCellOuttake,

    PowerCellHoodPointBlank,
    PowerCellHoodShort,
    PowerCellHoodMedium,
    PowerCellHoodLong,

    PowerCellKick,
    PowerCellKickerSpin,
    PowerCellKickerSpinReverse,
    PowerCellRotateCarousel,
    PowerCellFlywheelReverse,

    // Climber operations:
    ClimberExtend,
    ClimberRetract,
    ClimberHookLock,
    ClimberHookRelease,

    // Control Panel
    ControlPanelExtend,
    ControlPanelRetract,
}
