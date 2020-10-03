package frc.robot.common;

import frc.robot.TuningConstants;

public class AnglePair
{
    private double angle;
    private boolean swapDirection;

    /**
     * Initializes a new AnglePair
     * @param angle value to apply
     * @param swapDirection value to apply
     */
    public AnglePair(double angle, boolean swapDirection)
    {
        this.angle = angle;
        this.swapDirection = swapDirection;
    }

    /**
     * Get the closest angle equivalent to desiredAngle from current angle, swapping directions if it is closer
     * Note: prefers the same direction if equivalent
     * @param desiredAngle desired angle in degrees (between -180 and 180)
     * @param currentAngle current angle in degrees (any value)
     * @return pair containing closest angle fitting desired angle from current angle in degrees
     */
    public static AnglePair getClosestAngle(double desiredAngle, double currentAngle)
    {
        if (TuningConstants.THROW_EXCEPTIONS && 
            !Helpers.WithinRange(desiredAngle, -180.0, 180.0))
        {
            throw new RuntimeException(String.format("expect desiredAngle to be between (-180, 180). actual %f", desiredAngle));
        }

        double difference = ((desiredAngle % 360.0) - (currentAngle % 360.0)) % 360.0;
        AnglePair[] closeRotations = new AnglePair[3];
        if (difference >= 0.0)
        {
            closeRotations[0] = new AnglePair(currentAngle + difference, false);
            closeRotations[1] = new AnglePair(currentAngle + difference - 360.0, false);
            closeRotations[2] = new AnglePair(currentAngle + difference - 180.0, true);
        }
        else
        {
            closeRotations[0] = new AnglePair(currentAngle + difference, false);
            closeRotations[1] = new AnglePair(currentAngle + difference + 360.0, false);
            closeRotations[2] = new AnglePair(currentAngle + difference + 180.0, true);
        }

        AnglePair best = new AnglePair(currentAngle, false);
        double bestDistance = Double.POSITIVE_INFINITY;
        for (int i = 0; i < 3; i++)
        {
            AnglePair pair = closeRotations[i];
            if (pair != null)
            {
                double angleDistance = Math.abs(currentAngle - pair.getAngle());
                if (angleDistance < bestDistance)
                {
                    best = pair;
                    bestDistance = angleDistance;
                }
            }
        }

        return best;
    }

    public double getAngle()
    {
        return this.angle;
    }

    public boolean getSwapDirection()
    {
        return this.swapDirection;
    }
}
