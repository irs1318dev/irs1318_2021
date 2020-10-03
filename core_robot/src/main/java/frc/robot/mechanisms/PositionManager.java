package frc.robot.mechanisms;

import frc.robot.*;
import frc.robot.common.*;
import frc.robot.common.robotprovider.*;
import frc.robot.driver.AnalogOperation;
import frc.robot.driver.DigitalOperation;
import frc.robot.driver.common.Driver;

import com.google.inject.Inject;
import com.google.inject.Singleton;

/**
 * Position manager
 * 
 * This class maintains the approximate current location and orientation of the robot relative to its starting point.
 * This uses Jim's differential odometry algorithm and the NavX independently. In the future we can consider adding other sensors to help correct for error.
 * 
 */
@Singleton
public class PositionManager implements IMechanism
{
    private final ILogger logger;
    private final INavx navx;

    private Driver driver;

    private boolean navxIsConnected;

    // Position coordinates
    private double navxX;
    private double navxY;
    private double navxZ;

    // Orientation
    private double navxAngle;
    private double startAngle;
    private double resetAngle;

    /**
     * Initializes a new PositionManager
     * @param logger to use
     * @param provider for obtaining electronics objects
     */
    @Inject
    public PositionManager(
        LoggingManager logger,
        IRobotProvider provider)
    {
        this.logger = logger;
        this.navx = provider.getNavx();
        this.driver = null;

        this.navxIsConnected = false;

        this.navxX = 0.0;
        this.navxY = 0.0;
        this.navxZ = 0.0;

        this.navxAngle = 0.0;
        this.startAngle = 0.0;
        this.resetAngle = 0.0;
    }

    /**
     * set the driver that the mechanism should use
     * @param driver to use
     */
    @Override
    public void setDriver(Driver driver)
    {
        // At the beginning of autonomous, reset the position manager so that we consider ourself at the origin (0,0) and facing the 0 direction.
        this.driver = driver;
        if (this.driver.isAutonomous())
        {
            this.reset();
        }
    }

    /**
     * read all of the sensors for the mechanism that we will use in macros/autonomous mode and record their values
     */
    @Override
    public void readSensors()
    {
        this.navxIsConnected = this.navx.isConnected();

        this.navxAngle = -1.0 * this.navx.getAngle() - this.resetAngle;
        this.navxX = this.navx.getDisplacementX() * 100.0;
        this.navxY = this.navx.getDisplacementY() * 100.0;
        this.navxZ = this.navx.getDisplacementZ() * 100.0;

        // log the current position and orientation
        this.logger.logBoolean(LoggingKey.PositionNavxConnected, this.navxIsConnected);
        this.logger.logNumber(LoggingKey.PositionNavxAngle, this.navxAngle);
        this.logger.logNumber(LoggingKey.PositionNavxX, this.navxX);
        this.logger.logNumber(LoggingKey.PositionNavxY, this.navxY);
        this.logger.logNumber(LoggingKey.PositionNavxZ, this.navxZ);

        this.logger.logNumber(LoggingKey.PositionStartingAngle, this.startAngle);
    }

    /**
     * calculate the various outputs to use based on the inputs and apply them to the outputs for the relevant mechanism
     */
    @Override
    public void update()
    {
        double angle = this.driver.getAnalog(AnalogOperation.PositionStartingAngle);
        if (angle != 0.0)
        {
            this.startAngle = angle;
        }

        if (this.driver.getDigital(DigitalOperation.PositionResetFieldOrientation))
        {
            this.resetAngle = this.navxAngle;
        }
    }

    /**
     * stop the relevant component
     */
    @Override
    public void stop()
    {
        this.reset();
    }

    /**
     * Retrieve whether the navx is connected
     * @return whether the navx is connected
     */
    public boolean getNavxIsConnected()
    {
        return this.navxIsConnected;
    }

    /**
     * Retrieve the current angle (counter-clockwise) in degrees
     * @return the current angle in degrees
     */
    public double getNavxAngle()
    {
        return this.navxAngle + this.startAngle;
    }

    /**
     * Retrieve the current x position
     * @return the current x position
     */
    public double getNavxX()
    {
        return this.navxX;
    }

    /**
     * Retrieve the current y position
     * @return the current y position
     */
    public double getNavxY()
    {
        return this.navxY;
    }

    /**
     * Retrieve the current z position
     * @return the current z position
     */
    public double getNavxZ()
    {
        return this.navxZ;
    }

    /**
     * reset the position manager so it considers the current location to be "0"
     */
    public void reset()
    {
        this.navxX = 0.0;
        this.navxY = 0.0;
        this.navxZ = 0.0;

        this.navxAngle = 0.0;
        this.startAngle = 0.0;
        this.resetAngle = 0.0;

        this.navx.reset();
        this.navx.resetDisplacement();
    }
}
