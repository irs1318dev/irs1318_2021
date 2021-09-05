package frc.robot.driver.controltasks;

import frc.robot.driver.*;

/**
 * Task that applies the starting angle
 * 
 */
public class PositionStartingTask extends UpdateCycleTask
{
    private final double angle;
    private final boolean resetDriveTrain;
    private final boolean resetOrientation; 

    /**
     * Initializes a new PositionStartingTask
     * @param angle - offset to use from the default of facing away from the alliance driver station (in degrees)
     * @param resetDriveTrain - whether to reset the drivetrain wheels (to read from the absolute encoders)
     * @param resetOrientation - whether to reset the 
     */
    public PositionStartingTask(Double angle, boolean resetDriveTrain, boolean resetOrientation)
    {
        super(1);

        this.angle = angle;
        this.resetDriveTrain = resetDriveTrain;
        this.resetOrientation = resetOrientation;
    }

    /**
     * Begin the current task
     */
    @Override
    public void begin()
    {
        super.begin();

        this.setAnalogOperationState(AnalogOperation.PositionStartingAngle, this.angle);
        this.setDigitalOperationState(DigitalOperation.DriveTrainReset, this.resetDriveTrain);
        this.setDigitalOperationState(DigitalOperation.PositionResetFieldOrientation, this.resetOrientation);
    }

    /**
     * Run an iteration of the current task and apply any control changes 
     */
    @Override
    public void update()
    {
        super.update();

        this.setAnalogOperationState(AnalogOperation.PositionStartingAngle, this.angle);
        this.setDigitalOperationState(DigitalOperation.DriveTrainReset, this.resetDriveTrain);
        this.setDigitalOperationState(DigitalOperation.PositionResetFieldOrientation, this.resetOrientation);
    }

    /**
     * End the current task and reset control changes appropriately
     */
    @Override
    public void end()
    {
        super.end();

        this.setAnalogOperationState(AnalogOperation.PositionStartingAngle, 0.0);
        this.setDigitalOperationState(DigitalOperation.DriveTrainReset, false);
        this.setDigitalOperationState(DigitalOperation.PositionResetFieldOrientation, false);
    }
}
