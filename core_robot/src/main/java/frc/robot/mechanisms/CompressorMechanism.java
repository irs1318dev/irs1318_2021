package frc.robot.mechanisms;

import frc.robot.ElectronicsConstants;
import frc.robot.common.IMechanism;
import frc.robot.common.robotprovider.ICompressor;
import frc.robot.common.robotprovider.IRobotProvider;
import frc.robot.driver.DigitalOperation;
import frc.robot.driver.common.Driver;

import com.google.inject.Inject;
import com.google.inject.Singleton;

/**
 * Compressor mechanism.
 * The mechanism defines the logic that controls a mechanism given inputs and operator-requested actions, and 
 * translates those into the abstract functions that should be applied to the outputs.
 * 
 * @author Will
 *
 */
@Singleton
public class CompressorMechanism implements IMechanism
{
    private final ICompressor compressor;

    private Driver driver;
    private boolean isStarted;

    /**
     * Initializes a new CompressorMechanism
     * @param provider for obtaining electronics objects
     */
    @Inject
    public CompressorMechanism(IRobotProvider provider)
    {
        this.compressor = provider.getCompressor(ElectronicsConstants.PCM_A_MODULE);
        this.isStarted = false;
    }

    /**
     * set the driver that the mechanism should use
     * @param driver to use
     */
    @Override
    public void setDriver(Driver driver)
    {
        this.driver = driver;
    }

    /**
     * read all of the sensors for the mechanism that we will use in macros/autonomous mode and record their values
     */
    @Override
    public void readSensors()
    {
        // no sensors to read for this mechanism
    }

    /**
     * calculate the various outputs to use based on the inputs and apply them to the outputs for the relevant mechanism
     */
    @Override
    public void update()
    {
        if (this.driver.getDigital(DigitalOperation.CompressorForceDisable))
        {
            this.compressor.stop();
            this.isStarted = false;
        }
        else if (!this.isStarted)
        {
            this.compressor.start();
            this.isStarted = true;
        }
    }

    /**
     * stop the relevant mechanism
     */
    @Override
    public void stop()
    {
        this.compressor.stop();
        this.isStarted = false;
    }
}
