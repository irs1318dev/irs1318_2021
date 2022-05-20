package frc.robot.mechanisms;

import frc.robot.ElectronicsConstants;
import frc.robot.common.ComplementaryFilter;
import frc.robot.common.IMechanism;
import frc.robot.common.robotprovider.IPowerDistribution;
import frc.robot.common.robotprovider.IRobotProvider;
import frc.robot.driver.DigitalOperation;
import frc.robot.driver.common.IDriver;

import com.google.inject.Inject;
import com.google.inject.Singleton;

/**
 * Power manager.
 * 
 * @author Will
 *
 */
@Singleton
public class PowerManager implements IMechanism
{
    private final IDriver driver;
    private final IPowerDistribution powerDistribution;

    private ComplementaryFilter batteryVoltageFilter;

    /**
     * Initializes a new PowerManager
     * @param provider for obtaining electronics objects
     */
    @Inject
    public PowerManager(IDriver driver, IRobotProvider provider)
    {
        this.driver = driver;
        this.powerDistribution = provider.getPowerDistribution(ElectronicsConstants.POWER_DISTRIBUTION_CAN_ID, ElectronicsConstants.POWER_DISTRIBUTION_TYPE);
        this.batteryVoltageFilter = new ComplementaryFilter(0.4, 0.6, this.powerDistribution.getBatteryVoltage());
    }

    public double getCurrent(int pdpChannel)
    {
        return this.powerDistribution.getCurrent(pdpChannel);
    }

    public double getBatteryVoltage()
    {
        return this.batteryVoltageFilter.getValue();
    }

    @Override
    public void readSensors()
    {
        this.batteryVoltageFilter.update(this.powerDistribution.getBatteryVoltage());
    }

    @Override
    public void update()
    {
        // no outputs to update for this mechanism
    }

    @Override
    public void stop()
    {
        this.batteryVoltageFilter.reset();
    }
}
