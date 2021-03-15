package frc.robot.driver.controltasks;

import frc.robot.driver.DigitalOperation;

/**
 * Task that sets the Intake to spin
 */
public class IntakeOuttakeTask extends CompositeOperationTask
{
    private static DigitalOperation[] intakeOuttakeOperations =
    {
        DigitalOperation.PowerCellIntake,
        DigitalOperation.PowerCellOuttake,
    };

    /**
    * Initializes a new IntakeOuttakeTask
    * @param duration to run the intake for
    * @param intake in, or out
    */
    public IntakeOuttakeTask(double duration, boolean intake)
    {
        super(
            duration,
            intake ? DigitalOperation.PowerCellIntake : DigitalOperation.PowerCellOuttake,
            IntakeOuttakeTask.intakeOuttakeOperations);
    }
}
