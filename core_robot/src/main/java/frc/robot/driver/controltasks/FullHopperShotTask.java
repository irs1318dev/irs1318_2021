package frc.robot.driver.controltasks;

import frc.robot.driver.DigitalOperation;
import frc.robot.mechanisms.PowerCellMechanism;

public class FullHopperShotTask extends ControlTaskBase
{
    private PowerCellMechanism powerCellMechanism;

    public FullHopperShotTask()
    {
    }

    @Override
    public void begin()
    {
        this.powerCellMechanism = this.getInjector().getInstance(PowerCellMechanism.class);
    }

    @Override
    public void update()
    {
        this.setDigitalOperationState(DigitalOperation.PowerCellKick, true); //this.powerCellMechanism.isFlywheelSpunUp());
        this.setDigitalOperationState(DigitalOperation.PowerCellKickerSpin, true);
        this.setDigitalOperationState(DigitalOperation.PowerCellRotateCarousel, true);
    }

    @Override
    public void end()
    {
        this.setDigitalOperationState(DigitalOperation.PowerCellKick, false);
        this.setDigitalOperationState(DigitalOperation.PowerCellKickerSpin, false);
        this.setDigitalOperationState(DigitalOperation.PowerCellRotateCarousel, false);
    }

    @Override
    public boolean hasCompleted()
    {
        return false;
    }
}