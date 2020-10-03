package frc.robot.common;

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

    public static AnglePair getClosestAngle(double desiredAngle, double currentAngle)
    {
        double multiplicand = Math.floor(currentAngle / 360.0);

        AnglePair[] closeRotations =
        {
            new AnglePair(desiredAngle + 360.0 * multiplicand, false),
            new AnglePair(desiredAngle + 360.0 * multiplicand - 180.0, true),
            new AnglePair(desiredAngle + 360.0 * multiplicand + 180.0, true)
        };

        AnglePair best = new AnglePair(currentAngle, false);
        double bestDistance = Double.POSITIVE_INFINITY;
        for (int i = 0; i < 3; i++)
        {
            AnglePair pair = closeRotations[i];
            double angleDistance = Math.abs(currentAngle - pair.getAngle());
            if (angleDistance < bestDistance)
            {
                best = pair;
                bestDistance = angleDistance;
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
