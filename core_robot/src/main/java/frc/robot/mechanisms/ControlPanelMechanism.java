package frc.robot.mechanisms;

import frc.robot.*;
import frc.robot.common.*;
import frc.robot.common.robotprovider.*;
import frc.robot.driver.*;
import frc.robot.driver.common.*;

import com.google.inject.Inject;
import com.google.inject.Singleton;

@Singleton
public class ControlPanelMechanism implements IMechanism
{
    private final ILogger logger;

    private final IDoubleSolenoid extender;
    private final IVictorSPX spinnerMotor;

    private Driver driver;
    private boolean isExtended;


    @Inject
    public ControlPanelMechanism(LoggingManager logger, IRobotProvider provider)
    {
        this.logger = logger;


        this.extender = provider.getDoubleSolenoid(ElectronicsConstants.PCM_A_MODULE, ElectronicsConstants.CONTROLPANEL_EXTENDER_FORWARD_PCM, ElectronicsConstants.CONTROLPANEL_EXTENDER_REVERSE_PCM);

        this.spinnerMotor = provider.getVictorSPX(ElectronicsConstants.CONTROLPANEL_SPINNER_CAN_ID);
        this.spinnerMotor.setInvertOutput(HardwareConstants.CONTROLPANEL_SPINNER_INVERT_OUTPUT);
        this.spinnerMotor.setControlMode(TalonSRXControlMode.PercentOutput);
        this.spinnerMotor.setNeutralMode(MotorNeutralMode.Brake);

        this.isExtended = false;
    }

    @Override
    public void readSensors()
    {
    }

    @Override
    public void update()
    {
        if (this.driver.getDigital(DigitalOperation.ControlPanelExtend))
        {
            this.isExtended = true;
            this.extender.set(DoubleSolenoidValue.Forward);
        }
        else if (this.driver.getDigital(DigitalOperation.ControlPanelRetract))
        {
            this.isExtended = false;
            this.extender.set(DoubleSolenoidValue.Reverse);
        }

        this.logger.logBoolean(LoggingKey.ControlPanelExtend, this.isExtended);
        if (this.isExtended)
        {
            double speed = this.driver.getAnalog(AnalogOperation.ControlPanelSpinSpeed);
            this.spinnerMotor.set(speed);
        }
        else
        {
            this.spinnerMotor.stop();
        }
    }

    @Override
    public void stop()
    {
        this.extender.set(DoubleSolenoidValue.Off);
        this.spinnerMotor.stop();
    }

    @Override
    public void setDriver(Driver driver)
    {
        this.driver = driver;
    }
}