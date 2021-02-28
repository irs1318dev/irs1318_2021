package frc.robot.driver;

public enum DigitalOperation implements IOperation
{
    PositionResetFieldOrientation,

    // Vision operations:
    VisionForceDisable,
    VisionEnable,
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
}
