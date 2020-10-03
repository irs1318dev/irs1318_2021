package frc.robot.common;

import org.junit.jupiter.api.Test;

import org.junit.jupiter.api.Assertions;

public class AnglePairTests
{
    //@Test
    public void checkClosestAngle()
    {
        for (double goal = -180.0; goal <= 180.0; goal += 1.0)
        {
            for (double multiplier = -3.0; multiplier <= 3.0; multiplier += 1.0)
            {
                double expected = goal + multiplier * 360.0;
                for (double offset = -179.0; offset <= 179.0; offset += 1.0)
                {
                    AnglePair pair = AnglePair.getClosestAngle(goal, expected + offset);
                    if (offset < -90.0)
                    {
                        Assertions.assertEquals(
                            expected - 180.0,
                            pair.getAngle(),
                            String.format("%f %f %f", goal, multiplier, offset));

                        Assertions.assertEquals(true, pair.getSwapDirection());
                    }
                    else if (offset == -90.0)
                    {
                    }
                    else if (offset < 90.0)
                    {
                        Assertions.assertEquals(
                            expected,
                            pair.getAngle(),
                            String.format("%f %f %f", goal, multiplier, offset));

                        Assertions.assertEquals(false, pair.getSwapDirection());
                    }
                    else if (offset == 90.0)
                    {
                    }
                    else
                    {
                        Assertions.assertEquals(
                            expected + 180.0,
                            pair.getAngle(),
                            String.format("%f %f %f", goal, multiplier, offset));

                        Assertions.assertEquals(true, pair.getSwapDirection());
                    }
                }
            }
        }
    }

    @Test
    public void checkClosestAngle1()
    {
        AnglePair pair = AnglePair.getClosestAngle(270.0, 361.0);
        Assertions.assertEquals(
            450.0,
            pair.getAngle());

        Assertions.assertEquals(true, pair.getSwapDirection());
    }
}