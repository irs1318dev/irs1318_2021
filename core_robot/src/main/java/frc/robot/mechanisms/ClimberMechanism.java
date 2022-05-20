package frc.robot.mechanisms;

import frc.robot.*;
import frc.robot.common.*;
import frc.robot.common.robotprovider.*;
import frc.robot.driver.*;
import frc.robot.driver.common.*;

import com.google.inject.Inject;
import com.google.inject.Singleton;

@Singleton
public class ClimberMechanism implements IMechanism
{
    private final IDriver driver;

    private final IDoubleSolenoid climberExtendSolenoid;
    private final IDoubleSolenoid climberGrabSolenoid;

    private final ITalonSRX winchMotorMaster;

    private boolean isExtended;
    private boolean hasReleased;

    @Inject
    public ClimberMechanism(IDriver driver, IRobotProvider provider)
    {
        this.driver = driver;

        this.climberExtendSolenoid = provider.getDoubleSolenoid(ElectronicsConstants.PNEUMATICS_MODULE_B, ElectronicsConstants.PNEUMATICS_MODULE_TYPE_B, ElectronicsConstants.CLIMBER_EXTEND_FORWARD_PCM, ElectronicsConstants.CLIMBER_EXTEND_REVERSE_PCM);
        this.climberGrabSolenoid = provider.getDoubleSolenoid(ElectronicsConstants.PNEUMATICS_MODULE_A, ElectronicsConstants.PNEUMATICS_MODULE_TYPE_A, ElectronicsConstants.CLIMBER_GRAB_FORWARD_PCM, ElectronicsConstants.CLIMBER_GRAB_REVERSE_PCM);

        this.winchMotorMaster = provider.getTalonSRX(ElectronicsConstants.CLIMBER_WINCH_MASTER_CAN_ID);
        this.winchMotorMaster.setInvertOutput(HardwareConstants.CLIMBER_WINCH_MASTER_INVERT_OUTPUT);
        this.winchMotorMaster.setControlMode(TalonXControlMode.PercentOutput);
        this.winchMotorMaster.setNeutralMode(MotorNeutralMode.Brake);

        this.isExtended = false;
        this.hasReleased = false;
    }

    @Override
    public void readSensors()
    {
    }

    @Override
    public void update()
    {
        if (this.driver.getDigital(DigitalOperation.ClimberExtend))
        {
            this.isExtended = true;
            this.climberExtendSolenoid.set(DoubleSolenoidValue.Forward);
        }
        else if (this.driver.getDigital(DigitalOperation.ClimberRetract))
        {
            this.isExtended = false;
            this.climberExtendSolenoid.set(DoubleSolenoidValue.Reverse);
        }

        if (this.driver.getDigital(DigitalOperation.ClimberHookLock))
        {
            this.climberGrabSolenoid.set(DoubleSolenoidValue.Reverse);
        }
        else if (this.isExtended && this.driver.getDigital(DigitalOperation.ClimberHookRelease))
        {
            this.hasReleased = true;
            this.climberGrabSolenoid.set(DoubleSolenoidValue.Forward);
        }

        if (this.hasReleased)
        {
            double speed = this.driver.getAnalog(AnalogOperation.ClimberWinch);
            this.winchMotorMaster.set(speed);
        }
    }

    @Override
    public void stop()
    {
        this.climberExtendSolenoid.set(DoubleSolenoidValue.Off);
        this.climberGrabSolenoid.set(DoubleSolenoidValue.Off);
        this.winchMotorMaster.stop();
    }
}