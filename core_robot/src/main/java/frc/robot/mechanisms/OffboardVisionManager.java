package frc.robot.mechanisms;

import frc.robot.*;
import frc.robot.common.*;
import frc.robot.common.robotprovider.*;
import frc.robot.driver.*;
import frc.robot.driver.common.*;

import com.google.inject.Inject;
import com.google.inject.Singleton;

/**
 * Offboard Vision manager.
 * 
 * @author Will
 *
 */
@Singleton
public class OffboardVisionManager implements IMechanism
{
    private final INetworkTableProvider networkTable;
    private final ILogger logger;

    private final IDigitalOutput ringLight;

    private Driver driver;

    private double centerX;
    private double centerY;

    private Double distance;
    private Double horizontalAngle;

    private double powercellX;
    private double powercellY;

    /**
     * Initializes a new OffboardVisionManager
     * @param logger for logging to smart dashboard
     * @param provider for obtaining electronics objects
     */
    @Inject
    public OffboardVisionManager(LoggingManager logger, IRobotProvider provider)
    {
        this.logger = logger;

        this.networkTable = provider.getNetworkTableProvider();
        this.ringLight = provider.getDigitalOutput(ElectronicsConstants.VISION_RING_LIGHT_DIO);

        this.centerX = 0.0;
        this.centerY = 0.0;

        this.powercellX = 0.0;
        this.powercellY = 0.0;
    }

    /**
     * read all of the sensors for the mechanism that we will use in macros/autonomous mode and record their values
     */
    @Override
    public void readSensors()
    {
        this.centerX = this.networkTable.getSmartDashboardNumber("v.x");
        this.centerY = this.networkTable.getSmartDashboardNumber("v.y");

        this.powercellX = this.networkTable.getSmartDashboardNumber("v.pointX");
        this.powercellY = this.networkTable.getSmartDashboardNumber("v.pointY");

        this.logger.logNumber(LoggingKey.OffboardVisionX, this.centerX);
        this.logger.logNumber(LoggingKey.OffboardVisionY, this.centerY);

        this.logger.logNumber(LoggingKey.OffboardVisionPowercellX, this.powercellX);
        this.logger.logNumber(LoggingKey.OffboardVisionPowercellY, this.powercellY);

        // return if we couldn't find a vision target
        if (this.centerX < 0.0 || this.centerY < 0.0)
        {
            this.distance = null;
            this.horizontalAngle = null;

            //return;
        }

        if(this.distance != null && this.horizontalAngle != null) 
        {
            double yOffset = VisionConstants.LIFECAM_CAMERA_CENTER_WIDTH - this.centerY;
            double verticalAngle = Helpers.atand(yOffset / VisionConstants.LIFECAM_CAMERA_FOCAL_LENGTH_Y);

            this.distance = (HardwareConstants.CAMERA_TO_TARGET_Z_OFFSET / Helpers.tand(verticalAngle + HardwareConstants.CAMERA_PITCH)) - HardwareConstants.CAMERA_X_OFFSET;

            double xOffset = this.centerX - VisionConstants.LIFECAM_CAMERA_CENTER_WIDTH;
            this.horizontalAngle = Helpers.atand(xOffset / VisionConstants.LIFECAM_CAMERA_FOCAL_LENGTH_X) + HardwareConstants.CAMERA_YAW;

            this.logger.logNumber(LoggingKey.OffboardVisionDistance, this.distance);
            this.logger.logNumber(LoggingKey.OffboardVisionHorizontalAngle, this.horizontalAngle);
        }
    }

    @Override
    public void update()
    {
        boolean enableVision = this.driver.getDigital(DigitalOperation.VisionEnable) && !this.driver.getDigital(DigitalOperation.VisionForceDisable);
        boolean enableVideoStream = !this.driver.getDigital(DigitalOperation.VisionDisableOffboardStream);
        boolean enableVideoProcessing = !this.driver.getDigital(DigitalOperation.VisionDisableOffboardProcessing);
        this.logger.logBoolean(LoggingKey.OffboardVisionEnableVision, enableVision);
        this.logger.logBoolean(LoggingKey.OffboardVisionEnableStream, enableVideoStream);
        this.logger.logBoolean(LoggingKey.OffboardVisionEnableProcessing, enableVision && enableVideoProcessing);

        this.ringLight.set(enableVision);
    }

    @Override
    public void stop()
    {
        this.ringLight.set(false);

        this.logger.logBoolean(LoggingKey.OffboardVisionEnableVision, false);
        this.logger.logBoolean(LoggingKey.OffboardVisionEnableStream, false);
        this.logger.logBoolean(LoggingKey.OffboardVisionEnableProcessing, false);
    }

    @Override
    public void setDriver(Driver driver)
    {
        this.driver = driver;
    }

    public Double getHorizontalAngle()
    {
        return this.horizontalAngle;
    }

    public Double getDistance()
    {
        return this.distance;
    }

    public double getPowercellX() 
    {
        return this.powercellX;
    }

    public double getPowercellY() 
    {
        return this.powercellY;
    }
}
