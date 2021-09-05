package frc.robot.mechanisms;

import frc.robot.*;
import frc.robot.common.*;
import frc.robot.common.robotprovider.*;

import com.google.inject.Inject;
import com.google.inject.Singleton;

/**
 * Indicator Light manager
 * 
 * This class manages indicator lights on the robot.
 * 
 */
@Singleton
public class IndicatorLightManager implements IMechanism
{
    private enum LightMode
    {
        Off,
        Flashing,
        On,
    }

    private static final double FlashingFrequency = 0.5;
    private static final double FlashingComparisonFrequency = IndicatorLightManager.FlashingFrequency / 2.0;

    private final ITimer timer;

    private final IDigitalOutput xIndicator;

    @Inject
    public IndicatorLightManager(
        IRobotProvider provider,
        ITimer timer)
    {
        this.timer = timer;

        this.xIndicator = provider.getDigitalOutput(ElectronicsConstants.INDICATOR_LIGHT_X_DIO);
    }

    @Override
    public void readSensors()
    {
    }

    @Override
    public void update()
    {
        LightMode spunUpMode = LightMode.Off;
        if (true == false) // some condition
        {
            spunUpMode = LightMode.On;
        }

        this.controlLight(this.xIndicator, spunUpMode);
    }

    @Override
    public void stop()
    {
        this.xIndicator.set(false);
    }

    private void controlLight(IDigitalOutput indicatorLight, LightMode mode)
    {
        if (mode == LightMode.On)
        {
            indicatorLight.set(true);
        }
        else if (mode == LightMode.Off)
        {
            indicatorLight.set(false);
        }
        else
        {
            double currentTime = this.timer.get();
            if (currentTime % IndicatorLightManager.FlashingFrequency >= IndicatorLightManager.FlashingComparisonFrequency)
            {
                indicatorLight.set(true);
            }
            else
            {
                indicatorLight.set(false);
            }
        }
    }
}
