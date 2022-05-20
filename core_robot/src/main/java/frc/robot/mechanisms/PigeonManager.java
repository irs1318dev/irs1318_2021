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
 * Pigeon manager
 */
@Singleton
public class PigeonManager implements IPositionManager
{
    private final IDriver driver;
    private final ILogger logger;

    private final IPigeon2 pigeon;

    private final double[] ypr_deg; // shared array to avoid extra allocations

    private boolean isActive;

    // Orientation
    private double yaw;
    private double pitch;
    private double roll;

    private double startYaw;

    /**
     * Initializes a new PigeonManager
     * @param logger to use
     * @param provider for obtaining electronics objects
     */
    @Inject
    public PigeonManager(
        IDriver driver,
        LoggingManager logger,
        IRobotProvider provider)
    {
        this.driver = driver;
        this.logger = logger;

        this.pigeon = provider.getPigeon2(ElectronicsConstants.PIGEON_IMU_CAN_ID);
        this.pigeon.setYaw(0.0);

        this.ypr_deg = new double[3];

        this.isActive = false;

        this.yaw = 0.0;
        this.pitch = 0.0;
        this.roll = 0.0;

        this.startYaw = 0.0;
    }

    /**
     * read all of the sensors for the mechanism that we will use in macros/autonomous mode and record their values
     */
    @Override
    public void readSensors()
    {
        this.isActive = true;

        this.pigeon.getYawPitchRoll(this.ypr_deg);
        this.yaw = this.ypr_deg[0];
        this.pitch = this.ypr_deg[1];
        this.roll = this.ypr_deg[2];

        // log the current position and orientation
        ////this.logger.logBoolean(LoggingKey.PigeonState, this.isActive);
        this.logger.logNumber(LoggingKey.PigeonYaw, this.yaw);
        this.logger.logNumber(LoggingKey.PigeonPitch, this.pitch);
        this.logger.logNumber(LoggingKey.PigeonRoll, this.roll);

        this.logger.logNumber(LoggingKey.PigeonStartingYaw, this.startYaw);
    }

    /**
     * calculate the various outputs to use based on the inputs and apply them to the outputs for the relevant mechanism
     */
    @Override
    public void update()
    {
        double newYaw = this.driver.getAnalog(AnalogOperation.PositionStartingAngle);
        if (newYaw != 0.0)
        {
            this.startYaw = newYaw;
        }

        if (this.driver.getDigital(DigitalOperation.PositionResetFieldOrientation))
        {
            // clear the startAngle too if we are not actively setting it
            this.reset(newYaw == 0.0);
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
     * Retrieve whether the pigeon is connected
     * @return whether the pigeon is connected
     */
    public boolean getIsConnected()
    {
        return this.isActive;
    }

    /**
     * Retrieve the current angle (counter-clockwise) in degrees
     * @return the current angle in degrees
     */
    public double getAngle()
    {
        return this.yaw + this.startYaw;
    }

    /**
     * reset the position manager so it considers the current location to be "0"
     * @param resetStartAngle - whether to reset the start angle as well
     */
    public void reset(boolean resetStartAngle)
    {
        this.yaw = 0.0;
        this.pitch = 0.0;
        this.roll = 0.0;

        if (resetStartAngle)
        {
            this.startYaw = 0.0;
        }

        this.pigeon.setYaw(0.0);
    }
}
