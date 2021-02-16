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
 * Navx manager
 * 
 */
@Singleton
public class NavxManager implements IMechanism
{
    private final ILogger logger;
    private final INavx navx;

    private Driver driver;

    private boolean isConnected;

    // Position coordinates
    private double x;
    private double y;
    private double z;

    // Orientation
    private double angle;
    private double startAngle;

    /**
     * Initializes a new NavxManager
     * @param logger to use
     * @param provider for obtaining electronics objects
     */
    @Inject
    public NavxManager(
        LoggingManager logger,
        IRobotProvider provider)
    {
        this.logger = logger;
        this.navx = provider.getNavx();
        this.driver = null;

        this.isConnected = false;

        this.x = 0.0;
        this.y = 0.0;
        this.z = 0.0;

        this.angle = 0.0;
        this.startAngle = 0.0;
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
        this.isConnected = this.navx.isConnected();

        this.angle = -1.0 * this.navx.getAngle();
        this.x = this.navx.getDisplacementX() * 100.0;
        this.y = this.navx.getDisplacementY() * 100.0;
        this.z = this.navx.getDisplacementZ() * 100.0;

        // log the current position and orientation
        this.logger.logBoolean(LoggingKey.NavxConnected, this.isConnected);
        this.logger.logNumber(LoggingKey.NavxAngle, this.angle);
        this.logger.logNumber(LoggingKey.NavxX, this.x);
        this.logger.logNumber(LoggingKey.NavxY, this.y);
        this.logger.logNumber(LoggingKey.NavxZ, this.z);

        this.logger.logNumber(LoggingKey.NavxStartingAngle, this.startAngle);
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
            this.reset();
        }
    }

    /**
     * stop the relevant component
     */
    @Override
    public void stop()
    {
    }

    /**
     * Retrieve whether the navx is connected
     * @return whether the navx is connected
     */
    public boolean getIsConnected()
    {
        return this.isConnected;
    }

    /**
     * Retrieve the current angle (counter-clockwise) in degrees
     * @return the current angle in degrees
     */
    public double getAngle()
    {
        return this.angle;
    }

    /**
     * Retrieve the current x position
     * @return the current x position
     */
    public double getX()
    {
        return this.x;
    }

    /**
     * Retrieve the current y position
     * @return the current y position
     */
    public double getY()
    {
        return this.y;
    }

    /**
     * Retrieve the current z position
     * @return the current z position
     */
    public double getZ()
    {
        return this.z;
    }

    /**
     * reset the position manager so it considers the current location to be "0"
     */
    public void reset()
    {
        this.x = 0.0;
        this.y = 0.0;
        this.z = 0.0;

        this.angle = 0.0;
        this.startAngle = 0.0;

        this.navx.reset();
        this.navx.resetDisplacement();
    }
}
