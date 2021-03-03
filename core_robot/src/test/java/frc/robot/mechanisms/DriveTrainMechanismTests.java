package frc.robot.mechanisms;

// import static org.junit.jupiter.api.Assertions.assertEquals;

// import org.junit.jupiter.api.Test;

public class DriveTrainMechanismTests
{
    // private static double ONEINCHPERSECOND_IN100MS = 1357.5789;

    // @Test
    // public void testOdometryForward1()
    // {
    //     double angle = 0.0;
    //     double xPosition = 0.0;
    //     double yPosition = 0.0;
    //     double deltaNavxYaw = 0.0;
    //     double deltaT = 0.1;

    //     double[] steerAngles = new double[] { 0.0, 0.0, 0.0, 0.0 };
    //     double[] driveVelocities = new double[] { ONEINCHPERSECOND_IN100MS, ONEINCHPERSECOND_IN100MS, ONEINCHPERSECOND_IN100MS, ONEINCHPERSECOND_IN100MS };
    //     boolean[] isDirectionSwapped = new boolean[] { false, false, false, false };
    //     OdometryResult result = DriveTrainMechanism.calculateOdometry(angle, xPosition, yPosition, deltaNavxYaw, deltaT, steerAngles, driveVelocities, isDirectionSwapped);
    //     angle = result.angle;
    //     xPosition = result.xPosition;
    //     yPosition = result.yPosition;

    //     assertEquals(0.0, angle, 0.1);
    //     assertEquals(1.0, xPosition, 0.1);
    //     assertEquals(0.0, yPosition, 0.1);
    // }

    // @Test
    // public void testOdometryForward2()
    // {
    //     double angle = 0.0;
    //     double xPosition = 0.0;
    //     double yPosition = 0.0;
    //     double deltaNavxYaw = 0.0;
    //     double deltaT = 0.1;

    //     double[] steerAngles = new double[] { 0.0, 0.0, 0.0, 0.0 };
    //     double[] driveVelocities = new double[] { -ONEINCHPERSECOND_IN100MS, -ONEINCHPERSECOND_IN100MS, -ONEINCHPERSECOND_IN100MS, -ONEINCHPERSECOND_IN100MS };
    //     boolean[] isDirectionSwapped = new boolean[] { true, true, true, true };
    //     OdometryResult result = DriveTrainMechanism.calculateOdometry(angle, xPosition, yPosition, deltaNavxYaw, deltaT, steerAngles, driveVelocities, isDirectionSwapped);
    //     angle = result.angle;
    //     xPosition = result.xPosition;
    //     yPosition = result.yPosition;

    //     assertEquals(0.0, angle, 0.1);
    //     assertEquals(1.0, xPosition, 0.1);
    //     assertEquals(0.0, yPosition, 0.1);
    // }

    // @Test
    // public void testOdometryForward3()
    // {
    //     double angle = 0.0;
    //     double xPosition = 0.0;
    //     double yPosition = 0.0;
    //     double deltaNavxYaw = 0.0;
    //     double deltaT = 0.1;

    //     double[] steerAngles = new double[] { 0.0, 180.0, 0.0, 0.0 };
    //     double[] driveVelocities = new double[] { -ONEINCHPERSECOND_IN100MS, ONEINCHPERSECOND_IN100MS, -ONEINCHPERSECOND_IN100MS, ONEINCHPERSECOND_IN100MS };
    //     boolean[] isDirectionSwapped = new boolean[] { true, true, true, false };
    //     OdometryResult result = DriveTrainMechanism.calculateOdometry(angle, xPosition, yPosition, deltaNavxYaw, deltaT, steerAngles, driveVelocities, isDirectionSwapped);
    //     angle = result.angle;
    //     xPosition = result.xPosition;
    //     yPosition = result.yPosition;

    //     assertEquals(0.0, angle, 0.1);
    //     assertEquals(1.0, xPosition, 0.1);
    //     assertEquals(0.0, yPosition, 0.1);
    // }

    // @Test
    // public void testOdometryBackward1()
    // {
    //     double angle = 0.0;
    //     double xPosition = 0.0;
    //     double yPosition = 0.0;
    //     double deltaNavxYaw = 0.0;
    //     double deltaT = 0.1;

    //     double[] steerAngles = new double[] { 0.0, 0.0, 0.0, 0.0 };
    //     double[] driveVelocities = new double[] { ONEINCHPERSECOND_IN100MS, ONEINCHPERSECOND_IN100MS, ONEINCHPERSECOND_IN100MS, ONEINCHPERSECOND_IN100MS };
    //     boolean[] isDirectionSwapped = new boolean[] { true, true, true, true };
    //     OdometryResult result = DriveTrainMechanism.calculateOdometry(angle, xPosition, yPosition, deltaNavxYaw, deltaT, steerAngles, driveVelocities, isDirectionSwapped);
    //     angle = result.angle;
    //     xPosition = result.xPosition;
    //     yPosition = result.yPosition;

    //     assertEquals(0.0, angle, 0.1);
    //     assertEquals(-1.0, xPosition, 0.1);
    //     assertEquals(0.0, yPosition, 0.1);
    // }

    // @Test
    // public void testOdometryBackward2()
    // {
    //     double angle = 0.0;
    //     double xPosition = 0.0;
    //     double yPosition = 0.0;
    //     double deltaNavxYaw = 0.0;
    //     double deltaT = 0.1;

    //     double[] steerAngles = new double[] { 0.0, 0.0, 0.0, 0.0 };
    //     double[] driveVelocities = new double[] { -ONEINCHPERSECOND_IN100MS, -ONEINCHPERSECOND_IN100MS, ONEINCHPERSECOND_IN100MS, -ONEINCHPERSECOND_IN100MS };
    //     boolean[] isDirectionSwapped = new boolean[] { false, false, true, false };
    //     OdometryResult result = DriveTrainMechanism.calculateOdometry(angle, xPosition, yPosition, deltaNavxYaw, deltaT, steerAngles, driveVelocities, isDirectionSwapped);
    //     angle = result.angle;
    //     xPosition = result.xPosition;
    //     yPosition = result.yPosition;

    //     assertEquals(0.0, angle, 0.1);
    //     assertEquals(-1.0, xPosition, 0.1);
    //     assertEquals(0.0, yPosition, 0.1);
    // }

    // @Test
    // public void testOdometryBackward3()
    // {
    //     double angle = 0.0;
    //     double xPosition = 0.0;
    //     double yPosition = 0.0;
    //     double deltaNavxYaw = 0.0;
    //     double deltaT = 0.1;

    //     double[] steerAngles = new double[] { 180.0, 0.0, 0.0, 0.0 };
    //     double[] driveVelocities = new double[] { ONEINCHPERSECOND_IN100MS, ONEINCHPERSECOND_IN100MS, ONEINCHPERSECOND_IN100MS, -ONEINCHPERSECOND_IN100MS };
    //     boolean[] isDirectionSwapped = new boolean[] { false, true, true, false };
    //     OdometryResult result = DriveTrainMechanism.calculateOdometry(angle, xPosition, yPosition, deltaNavxYaw, deltaT, steerAngles, driveVelocities, isDirectionSwapped);
    //     angle = result.angle;
    //     xPosition = result.xPosition;
    //     yPosition = result.yPosition;

    //     assertEquals(0.0, angle, 0.1);
    //     assertEquals(-1.0, xPosition, 0.1);
    //     assertEquals(0.0, yPosition, 0.1);
    // }

    // @Test
    // public void testOdometryLeft1()
    // {
    //     double angle = 0.0;
    //     double xPosition = 0.0;
    //     double yPosition = 0.0;
    //     double deltaNavxYaw = 0.0;
    //     double deltaT = 0.1;

    //     double[] steerAngles = new double[] { 90.0, 90.0, 90.0, 90.0 };
    //     double[] driveVelocities = new double[] { ONEINCHPERSECOND_IN100MS, ONEINCHPERSECOND_IN100MS, ONEINCHPERSECOND_IN100MS, ONEINCHPERSECOND_IN100MS };
    //     boolean[] isDirectionSwapped = new boolean[] { false, false, false, false };
    //     OdometryResult result = DriveTrainMechanism.calculateOdometry(angle, xPosition, yPosition, deltaNavxYaw, deltaT, steerAngles, driveVelocities, isDirectionSwapped);
    //     angle = result.angle;
    //     xPosition = result.xPosition;
    //     yPosition = result.yPosition;

    //     assertEquals(0.0, angle, 0.1);
    //     assertEquals(0.0, xPosition, 0.1);
    //     assertEquals(1.0, yPosition, 0.1);
    // }

    // @Test
    // public void testOdometryLeft2()
    // {
    //     double angle = 0.0;
    //     double xPosition = 0.0;
    //     double yPosition = 0.0;
    //     double deltaNavxYaw = 0.0;
    //     double deltaT = 0.1;

    //     double[] steerAngles = new double[] { 90.0, 90.0, 90.0, 90.0 };
    //     double[] driveVelocities = new double[] { -ONEINCHPERSECOND_IN100MS, -ONEINCHPERSECOND_IN100MS, -ONEINCHPERSECOND_IN100MS, -ONEINCHPERSECOND_IN100MS };
    //     boolean[] isDirectionSwapped = new boolean[] { true, true, true, true };
    //     OdometryResult result = DriveTrainMechanism.calculateOdometry(angle, xPosition, yPosition, deltaNavxYaw, deltaT, steerAngles, driveVelocities, isDirectionSwapped);
    //     angle = result.angle;
    //     xPosition = result.xPosition;
    //     yPosition = result.yPosition;

    //     assertEquals(0.0, angle, 0.1);
    //     assertEquals(0.0, xPosition, 0.1);
    //     assertEquals(1.0, yPosition, 0.1);
    // }

    // @Test
    // public void testOdometryLeft3()
    // {
    //     double angle = 0.0;
    //     double xPosition = 0.0;
    //     double yPosition = 0.0;
    //     double deltaNavxYaw = 0.0;
    //     double deltaT = 0.1;

    //     double[] steerAngles = new double[] { 90.0, 90.0, -90.0, 90.0 };
    //     double[] driveVelocities = new double[] { -ONEINCHPERSECOND_IN100MS, ONEINCHPERSECOND_IN100MS, ONEINCHPERSECOND_IN100MS, ONEINCHPERSECOND_IN100MS };
    //     boolean[] isDirectionSwapped = new boolean[] { true, false, true, false };
    //     OdometryResult result = DriveTrainMechanism.calculateOdometry(angle, xPosition, yPosition, deltaNavxYaw, deltaT, steerAngles, driveVelocities, isDirectionSwapped);
    //     angle = result.angle;
    //     xPosition = result.xPosition;
    //     yPosition = result.yPosition;

    //     assertEquals(0.0, angle, 0.1);
    //     assertEquals(0.0, xPosition, 0.1);
    //     assertEquals(1.0, yPosition, 0.1);
    // }

    // @Test
    // public void testOdometryRight1()
    // {
    //     double angle = 0.0;
    //     double xPosition = 0.0;
    //     double yPosition = 0.0;
    //     double deltaNavxYaw = 0.0;
    //     double deltaT = 0.1;

    //     double[] steerAngles = new double[] { -90.0, -90.0, -90.0, -90.0 };
    //     double[] driveVelocities = new double[] { ONEINCHPERSECOND_IN100MS, ONEINCHPERSECOND_IN100MS, ONEINCHPERSECOND_IN100MS, ONEINCHPERSECOND_IN100MS };
    //     boolean[] isDirectionSwapped = new boolean[] { false, false, false, false };
    //     OdometryResult result = DriveTrainMechanism.calculateOdometry(angle, xPosition, yPosition, deltaNavxYaw, deltaT, steerAngles, driveVelocities, isDirectionSwapped);
    //     angle = result.angle;
    //     xPosition = result.xPosition;
    //     yPosition = result.yPosition;

    //     assertEquals(0.0, angle, 0.1);
    //     assertEquals(0.0, xPosition, 0.1);
    //     assertEquals(-1.0, yPosition, 0.1);
    // }

    // @Test
    // public void testOdometryRight2()
    // {
    //     double angle = 0.0;
    //     double xPosition = 0.0;
    //     double yPosition = 0.0;
    //     double deltaNavxYaw = 0.0;
    //     double deltaT = 0.1;

    //     double[] steerAngles = new double[] { -90.0, -90.0, 90.0, -90.0 };
    //     double[] driveVelocities = new double[] { -ONEINCHPERSECOND_IN100MS, -ONEINCHPERSECOND_IN100MS, ONEINCHPERSECOND_IN100MS, -ONEINCHPERSECOND_IN100MS };
    //     boolean[] isDirectionSwapped = new boolean[] { true, true, true, true };
    //     OdometryResult result = DriveTrainMechanism.calculateOdometry(angle, xPosition, yPosition, deltaNavxYaw, deltaT, steerAngles, driveVelocities, isDirectionSwapped);
    //     angle = result.angle;
    //     xPosition = result.xPosition;
    //     yPosition = result.yPosition;

    //     assertEquals(0.0, angle, 0.1);
    //     assertEquals(0.0, xPosition, 0.1);
    //     assertEquals(-1.0, yPosition, 0.1);
    // }

    // @Test
    // public void testOdometryRigh3()
    // {
    //     double angle = 0.0;
    //     double xPosition = 0.0;
    //     double yPosition = 0.0;
    //     double deltaNavxYaw = 0.0;
    //     double deltaT = 0.1;

    //     double[] steerAngles = new double[] { -90.0, -90.0, 90.0, -90.0 };
    //     double[] driveVelocities = new double[] { -ONEINCHPERSECOND_IN100MS, ONEINCHPERSECOND_IN100MS, ONEINCHPERSECOND_IN100MS, ONEINCHPERSECOND_IN100MS };
    //     boolean[] isDirectionSwapped = new boolean[] { true, false, true, false };
    //     OdometryResult result = DriveTrainMechanism.calculateOdometry(angle, xPosition, yPosition, deltaNavxYaw, deltaT, steerAngles, driveVelocities, isDirectionSwapped);
    //     angle = result.angle;
    //     xPosition = result.xPosition;
    //     yPosition = result.yPosition;

    //     assertEquals(0.0, angle, 0.1);
    //     assertEquals(0.0, xPosition, 0.1);
    //     assertEquals(-1.0, yPosition, 0.1);
    // }

    // @Test
    // public void testOdometryUpLeft()
    // {
    //     double angle = 0.0;
    //     double xPosition = 0.0;
    //     double yPosition = 0.0;
    //     double deltaNavxYaw = 0.0;
    //     double deltaT = 0.1;

    //     double[] steerAngles = new double[] { 45.0, 45.0, -135.0, 45.0 };
    //     double[] driveVelocities = new double[] { ONEINCHPERSECOND_IN100MS, ONEINCHPERSECOND_IN100MS, ONEINCHPERSECOND_IN100MS, ONEINCHPERSECOND_IN100MS };
    //     boolean[] isDirectionSwapped = new boolean[] { false, false, true, false };
    //     OdometryResult result = DriveTrainMechanism.calculateOdometry(angle, xPosition, yPosition, deltaNavxYaw, deltaT, steerAngles, driveVelocities, isDirectionSwapped);
    //     angle = result.angle;
    //     xPosition = result.xPosition;
    //     yPosition = result.yPosition;

    //     assertEquals(0.0, angle, 0.1);
    //     assertEquals(Math.sqrt(0.5), xPosition, 0.1);
    //     assertEquals(Math.sqrt(0.5), yPosition, 0.1);
    // }

    // @Test
    // public void testOdometryUpRight()
    // {
    //     double angle = 0.0;
    //     double xPosition = 0.0;
    //     double yPosition = 0.0;
    //     double deltaNavxYaw = 0.0;
    //     double deltaT = 0.1;

    //     double[] steerAngles = new double[] { -45.0, -45.0, 135.0, -45.0 };
    //     double[] driveVelocities = new double[] { -ONEINCHPERSECOND_IN100MS, -ONEINCHPERSECOND_IN100MS, ONEINCHPERSECOND_IN100MS, -ONEINCHPERSECOND_IN100MS };
    //     boolean[] isDirectionSwapped = new boolean[] { true, true, true, true };
    //     OdometryResult result = DriveTrainMechanism.calculateOdometry(angle, xPosition, yPosition, deltaNavxYaw, deltaT, steerAngles, driveVelocities, isDirectionSwapped);
    //     angle = result.angle;
    //     xPosition = result.xPosition;
    //     yPosition = result.yPosition;

    //     assertEquals(0.0, angle, 0.1);
    //     assertEquals(Math.sqrt(0.5), xPosition, 0.1);
    //     assertEquals(-Math.sqrt(0.5), yPosition, 0.1);
    // }

    // @Test
    // public void testOdometryBackRight()
    // {
    //     double angle = 0.0;
    //     double xPosition = 0.0;
    //     double yPosition = 0.0;
    //     double deltaNavxYaw = 0.0;
    //     double deltaT = 0.1;

    //     double[] steerAngles = new double[] { -135.0, -135.0, 45.0, -135.0 };
    //     double[] driveVelocities = new double[] { -ONEINCHPERSECOND_IN100MS, ONEINCHPERSECOND_IN100MS, ONEINCHPERSECOND_IN100MS, ONEINCHPERSECOND_IN100MS };
    //     boolean[] isDirectionSwapped = new boolean[] { true, false, true, false };
    //     OdometryResult result = DriveTrainMechanism.calculateOdometry(angle, xPosition, yPosition, deltaNavxYaw, deltaT, steerAngles, driveVelocities, isDirectionSwapped);
    //     angle = result.angle;
    //     xPosition = result.xPosition;
    //     yPosition = result.yPosition;

    //     assertEquals(0.0, angle, 0.1);
    //     assertEquals(-Math.sqrt(0.5), xPosition, 0.1);
    //     assertEquals(-Math.sqrt(0.5), yPosition, 0.1);
    // }

    // @Test
    // public void testOdometryTurnInPlaceCCW()
    // {
    //     double angle = 0.0;
    //     double xPosition = 0.0;
    //     double yPosition = 0.0;
    //     double deltaNavxYaw = 0.0;
    //     double deltaT = 0.1;

    //     double[] steerAngles = new double[] { 45.0, 135.0, -135.0, -45.0 };
    //     double[] driveVelocities = new double[] { ONEINCHPERSECOND_IN100MS, ONEINCHPERSECOND_IN100MS, ONEINCHPERSECOND_IN100MS, ONEINCHPERSECOND_IN100MS };
    //     boolean[] isDirectionSwapped = new boolean[] { false, false, false, false };
    //     OdometryResult result = DriveTrainMechanism.calculateOdometry(angle, xPosition, yPosition, deltaNavxYaw, deltaT, steerAngles, driveVelocities, isDirectionSwapped);
    //     angle = result.angle;
    //     xPosition = result.xPosition;
    //     yPosition = result.yPosition;

    //     assertEquals(0.0, angle, 0.1);
    //     assertEquals(0.0, xPosition, 0.1);
    //     assertEquals(0.0, yPosition, 0.1);
    // }
}
