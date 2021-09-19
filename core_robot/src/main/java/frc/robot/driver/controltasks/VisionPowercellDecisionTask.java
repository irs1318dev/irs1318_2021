package frc.robot.driver.controltasks;

import frc.robot.*;
import frc.robot.driver.DigitalOperation;
import frc.robot.driver.common.IControlTask;
import frc.robot.mechanisms.OffboardVisionManager;

public class VisionPowercellDecisionTask extends DecisionSequentialTask
{
    private IControlTask routineBlue;
    private IControlTask routineRed;

    //private Driver driver;

    private OffboardVisionManager visionManager;

    /**
     * Appends either the red or blue path depending on how far the nearest powercell is
     * @param routineBlue 
     * @param routineRed
     */
    public VisionPowercellDecisionTask(IControlTask routineBlue, IControlTask routineRed)
    {
        this.routineBlue = routineBlue;
        this.routineRed = routineRed;
    }

    @Override
    public void begin()
    {
        this.visionManager = this.getInjector().getInstance(OffboardVisionManager.class);

        this.setDigitalOperationState(DigitalOperation.VisionDisableStream, false);
        this.setDigitalOperationState(DigitalOperation.VisionEnablePowercellProcessing, true);

        //System.out.println(visionManager.getPowercellY() + " VISION Y VALUE");
        //System.out.println(this.driver.getDigital(DigitalOperation.VisionForceDisable));
        if (visionManager.getPowercellY() <= VisionConstants.VISION_POWERCELL_LOCATION_Y)
        {
            this.AppendTask(routineBlue);
        }
        else
        {
            this.AppendTask(routineRed);
        }
    }
}