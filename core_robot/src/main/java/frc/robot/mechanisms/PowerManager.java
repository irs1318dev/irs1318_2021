package frc.robot.mechanisms;

import frc.robot.common.ComplementaryFilter;
import frc.robot.common.IMechanism;
import frc.robot.common.robotprovider.IPowerDistributionPanel;
import frc.robot.common.robotprovider.IRobotProvider;

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
    private final IPowerDistributionPanel pdp;

    private ComplementaryFilter batteryVoltageFilter;

    /**
     * Initializes a new PowerManager
     * @param provider for obtaining electronics objects
     */
    @Inject
    public PowerManager(IRobotProvider provider)
    {
        this.pdp = provider.getPDP();
        this.batteryVoltageFilter = new ComplementaryFilter(0.4, 0.6, this.pdp.getBatteryVoltage());
    }

    public double getCurrent(int pdpChannel)
    {
        return this.pdp.getCurrent(pdpChannel);
    }

    public double getBatteryVoltage()
    {
        return this.batteryVoltageFilter.getValue();
    }

    @Override
    public void readSensors()
    {
        this.batteryVoltageFilter.update(this.pdp.getBatteryVoltage());
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
