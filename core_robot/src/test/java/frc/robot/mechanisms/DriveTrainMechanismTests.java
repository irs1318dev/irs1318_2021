package frc.robot.mechanisms;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Assertions;

public class DriveTrainMechanismTests
{
    @Test
    public void checkClosestAngle()
    {
        for (double goal = -180.0; goal <= 180.0; goal += 1.0)
        {
            for (double multiplier = -3.0; multiplier <= 3.0; multiplier += 1.0)
            {
                double expected = goal + multiplier * 360.0;
                for (double offset = -179.0; offset <= 179.0; offset += 1.0)
                {
                    Assertions.assertEquals(
                        expected,
                        DriveTrainMechanism.getClosestAngle(goal, expected + offset));
                }
            }
        }
    }
}
