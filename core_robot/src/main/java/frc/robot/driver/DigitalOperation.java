package frc.robot.driver;

public enum DigitalOperation implements IOperation
{
    PositionResetFieldOrientation,

    // Vision operations:
    VisionForceDisable,
    VisionEnableStream,
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
    PowerCellMoveToNextSlotInator,
    PowerCellMoveToPreviousSlot,
}
