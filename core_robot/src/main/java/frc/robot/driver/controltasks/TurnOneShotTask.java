package frc.robot.driver.controltasks;

import frc.robot.HardwareConstants;

/**
 * Task that turns the robot a certain amount clockwise or counterclockwise in-place using Positional PID.
 * 
 */
public class TurnOneShotTask extends MoveDistanceOneShotTaskBase
{
    private final double degrees;

    /**
     * Initializes a new TurnTask
     * @param degrees from the current orientation to rotate (positive means turn right/clockwise, negative means turn left/counter-clockwise)
     */
    public TurnOneShotTask(double degrees)
    {
        this(degrees, true);
    }

    /**
     * Initializes a new TurnTask
     * @param degrees from the current orientation to rotate (positive means turn right/clockwise, negative means turn left/counter-clockwise)
     */
    public TurnOneShotTask(double degrees, boolean resetPositionOnEnd)
    {
        super(resetPositionOnEnd);

        this.degrees = degrees;
    }

    /**
     * Determine the final encoder distance
     */
    @Override
    protected void determineFinalEncoderDistance()
    {
        double arcLength = Math.PI * HardwareConstants.DRIVETRAIN_WHEEL_SEPARATION_DISTANCE * (this.degrees / 360.0);
        this.desiredFinalLeftTicks = this.startLeftTicks + arcLength / HardwareConstants.DRIVETRAIN_LEFT_PULSE_DISTANCE;
        this.desiredFinalRightTicks = this.startRightTicks - arcLength / HardwareConstants.DRIVETRAIN_RIGHT_PULSE_DISTANCE;
    }
}
