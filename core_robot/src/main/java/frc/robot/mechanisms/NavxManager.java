package frc.robot.mechanisms;

import frc.robot.*;
import frc.robot.common.*;
import frc.robot.common.robotprovider.*;
import frc.robot.driver.AnalogOperation;
import frc.robot.driver.DigitalOperation;
import frc.robot.driver.common.IDriver;

import com.google.inject.Inject;
import com.google.inject.Singleton;

/**
 * Navx manager
 */
@Singleton
public class NavxManager implements IMechanism
{
    private final IDriver driver;
    private final ILogger logger;

    private final INavx navx;

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
        IDriver driver,
        LoggingManager logger,
        IRobotProvider provider)
    {
        this.driver = driver;
        this.logger = logger;

        this.navx = provider.getNavx();

        this.isConnected = false;

        this.x = 0.0;
        this.y = 0.0;
        this.z = 0.0;

        this.angle = 0.0;
        this.startAngle = 0.0;
    }

    /**
     * read all of the sensors for the mechanism that we will use in macros/autonomous mode and record their values
     */
    @Override
    public void readSensors()
    {
        this.isConnected = this.navx.isConnected();

        double pitch = this.navx.getPitch();
        double roll = this.navx.getRoll();
        double yaw = this.navx.getYaw();
        double angle = this.navx.getAngle();
        this.angle = -1.0 * angle;
        this.x = this.navx.getDisplacementX() * 100.0;
        this.y = this.navx.getDisplacementY() * 100.0;
        this.z = this.navx.getDisplacementZ() * 100.0;

        // log the current position and orientation
        this.logger.logBoolean(LoggingKey.NavxConnected, this.isConnected);
        this.logger.logNumber(LoggingKey.NavxAngle, this.angle);
        this.logger.logNumber(LoggingKey.NavxPitch, pitch);
        this.logger.logNumber(LoggingKey.NavxRoll, roll);
        this.logger.logNumber(LoggingKey.NavxYaw, yaw);
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
            // clear the startAngle too if we are not actively setting it
            this.reset(angle == 0.0);
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
        return this.angle + this.startAngle;
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
     * @param resetStartAngle - whether to reset the start angle as well
     */
    public void reset(boolean resetStartAngle)
    {
        this.x = 0.0;
        this.y = 0.0;
        this.z = 0.0;

        this.angle = 0.0;
        if (resetStartAngle)
        {
            this.startAngle = 0.0;
        }

        this.navx.reset();
        this.navx.resetDisplacement();
    }
}
