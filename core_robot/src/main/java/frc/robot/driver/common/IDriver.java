package frc.robot.driver.common;

import frc.robot.driver.*;

/**
 * Interface describing the Driver that operates the robot.  This is either autonomous or teleop/user driver.
 *
 */
public interface IDriver
{
    /**
     * Checks whether the driver is in autonomous mode
     */
    public boolean isAutonomous();

    /**
     * Tell the driver that some time has passed
     */
    public void update();

    /**
     * Tell the driver that robot operation is stopping
     */
    public void stop();

    /**
     * Starts the autonomous period of the match (e.g. begins auto routine)
     */
    public void startAutonomous();

    /**
     * Get a boolean indicating whether the current digital operation is enabled
     * @param digitalOperation to get
     * @return the current value of the digital operation
     */
    public boolean getDigital(DigitalOperation digitalOperation);

    /**
     * Get a double between -1.0 and 1.0 indicating the current value of the analog operation
     * @param analogOperation to get
     * @return the current value of the analog operation
     */
    public double getAnalog(AnalogOperation analogOperation);
}
