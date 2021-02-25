import frc.robot.*;
import frc.robot.driver.common.IControlTask;
import frc.robot.mechanisms.OffboardVisionManager;

public class VisionPowercellTask extends DecisionSequentialTask {
    
    private IControlTask routineBlue;
    private IControlTask routineRed;
    
    protected OffboardVisionManager visionManager;
    /**
     * Appends either the red or blue path depending on how far the nearest powercell is
     * @param routineBlue 
     * @param routineRed
     */
    public VisionPowercellTask(IControlTask routineBlue, IControlTask routineRed)
    {
        this.routineBlue = routineBlue;
        this.routineRed = routineRed;
    }

    @Override
    public void begin()
    {
        this.visionManager = this.getInjector().getInstance(OffboardVisionManager.class);

        if (visionManager.getPowercellY() >= VisionConstants.VISION_POWERCELL_LOCATION_Y)
        {
            this.AppendTask(routineBlue);
        }
        else
        {
            this.AppendTask(routineRed);
        }
    }

} // yaaaaaAAAaaaAaaaAAAAaa