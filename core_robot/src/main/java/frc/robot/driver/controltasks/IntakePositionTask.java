package frc.robot.driver.controltasks;

import frc.robot.driver.DigitalOperation;

/**
 * Task that sets the Intake Position.
 */
public class IntakePositionTask extends CompositeOperationTask
{
    private static DigitalOperation[] intakePositionOperations =
    {
        DigitalOperation.PowerCellIntakeExtend,
        DigitalOperation.PowerCellIntakeRetract,
    };

    /**
    * Initializes a new IntakePositionTask
    * @param extend or retract
    */
    public IntakePositionTask(boolean extend)
    {
        super(
            0.25,
            extend ? DigitalOperation.PowerCellIntakeExtend : DigitalOperation.PowerCellIntakeRetract, 
            IntakePositionTask.intakePositionOperations);
    }
}
