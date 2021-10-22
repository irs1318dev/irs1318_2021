package frc.robot.driver.controltasks;

import frc.robot.driver.DigitalOperation;

public class DriveTrainFieldOrientationModeTask extends CompositeOperationTask
{
    private static final DigitalOperation[] ORIENTATION =
        new DigitalOperation[]
        {
            DigitalOperation.DriveTrainEnableFieldOrientation,
            DigitalOperation.DriveTrainDisableFieldOrientation,
        };

    /**
     * Initializes a new FlywheelFixedSpinTask
     * @param enable
     */
    public DriveTrainFieldOrientationModeTask(boolean enable)
    {
        super(0.1, enable ? DigitalOperation.DriveTrainEnableFieldOrientation : DigitalOperation.DriveTrainDisableFieldOrientation, ORIENTATION);
    }
}
