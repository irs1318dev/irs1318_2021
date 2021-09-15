package frc.robot.mechanisms;

import frc.robot.*;
import frc.robot.common.*;
import frc.robot.common.robotprovider.*;
import frc.robot.driver.*;
import frc.robot.driver.common.*;

import com.google.inject.Inject;
import com.google.inject.Singleton;

/**
 * Extends/retracts intake and rolls the rollers in and out
 * 
 * @author Will, kwen perper, beezy choofs
 */
@Singleton
public class PowerCellMechanism implements IMechanism
{
    private static final boolean HAS_CAROUSEL_MOTOR = true;
    private static final boolean HAS_INTAKE_MOTOR = true;
    private static final boolean HAS_KICKER_MOTOR = true;
    private static final boolean HAS_SHOOTER_MOTOR = true;
    private static final boolean HAS_PNEUMATICS = true;

    private static final int slotId = 0;

    private final IDriver driver;
    private final ILogger logger;

    private final IDoubleSolenoid intakeSolenoid;
    private final ISparkMax rollerMotor;

    private final IDoubleSolenoid outerHood;
    private final IDoubleSolenoid innerHood;
    private final ITalonSRX flyWheel;

    private final IVictorSPX carouselMotor;

    private final IDoubleSolenoid kickerSolenoid;
    private final ITalonSRX kickerMotor;

    private double flywheelPosition;
    private double flywheelVelocity;
    private double flywheelError;

    private boolean intakeExtended;

    private double flywheelVelocitySetpoint;

    @Inject
    public PowerCellMechanism(IDriver driver, LoggingManager logger, IRobotProvider provider)
    {
        this.driver = driver;
        this.logger = logger;

        // intake components:
        if (PowerCellMechanism.HAS_PNEUMATICS)
        {
            this.intakeSolenoid = provider.getDoubleSolenoid(ElectronicsConstants.PCM_A_MODULE, ElectronicsConstants.POWERCELL_INTAKE_FORWARD_PCM, ElectronicsConstants.POWERCELL_INTAKE_REVERSE_PCM);
        }
        else
        {
            this.intakeSolenoid = null;
        }

        if (PowerCellMechanism.HAS_INTAKE_MOTOR)
        {
            this.rollerMotor = provider.getSparkMax(ElectronicsConstants.POWERCELL_ROLLER_MOTOR_CAN_ID, SparkMaxMotorType.Brushless);
            this.rollerMotor.setInvertOutput(HardwareConstants.POWERCELL_ROLLER_MOTOR_INVERT_OUTPUT);
            this.rollerMotor.setControlMode(SparkMaxControlMode.PercentOutput);
            this.rollerMotor.setNeutralMode(MotorNeutralMode.Brake);
        }
        else
        {
            this.rollerMotor = null;
        }

        // shooter components:
        if (PowerCellMechanism.HAS_PNEUMATICS)
        {
            this.outerHood = provider.getDoubleSolenoid(ElectronicsConstants.PCM_B_MODULE, ElectronicsConstants.POWERCELL_OUTER_HOOD_FORWARD_PCM, ElectronicsConstants.POWERCELL_OUTER_HOOD_REVERSE_PCM);
            this.innerHood = provider.getDoubleSolenoid(ElectronicsConstants.PCM_B_MODULE, ElectronicsConstants.POWERCELL_INNER_HOOD_FORWARD_PCM, ElectronicsConstants.POWERCELL_INNER_HOOD_REVERSE_PCM);
        }
        else
        {
            this.outerHood = null;
            this.innerHood = null;
        }

        if (PowerCellMechanism.HAS_SHOOTER_MOTOR)
        {
            this.flyWheel = provider.getTalonSRX(ElectronicsConstants.POWERCELL_FLYWHEEL_MASTER_CAN_ID);
            this.flyWheel.setInvertOutput(HardwareConstants.POWERCELL_FLYWHEEL_MASTER_INVERT_OUTPUT);
            this.flyWheel.setInvertSensor(HardwareConstants.POWERCELL_FLYWHEEL_MASTER_INVERT_SENSOR);
            this.flyWheel.setNeutralMode(MotorNeutralMode.Coast);
            this.flyWheel.setSensorType(TalonXFeedbackDevice.QuadEncoder);
            this.flyWheel.setPosition(0);
            this.flyWheel.setControlMode(TalonSRXControlMode.Velocity);
            this.flyWheel.setPIDF(
                TuningConstants.POWERCELL_FLYWHEEL_ONE_VELOCITY_PID_KP,
                TuningConstants.POWERCELL_FLYWHEEL_ONE_VELOCITY_PID_KI,
                TuningConstants.POWERCELL_FLYWHEEL_ONE_VELOCITY_PID_KD,
                TuningConstants.POWERCELL_FLYWHEEL_ONE_VELOCITY_PID_KF,
                PowerCellMechanism.slotId);
            this.flyWheel.configureVelocityMeasurements(TuningConstants.POWERCELL_FLYWHEEL_VELOCITY_PERIOD, TuningConstants.POWERCELL_FLYWHEEL_VELOCITY_WINDOWSIZE);
            this.flyWheel.setVoltageCompensation(TuningConstants.POWERCELL_FLYWHEEL_MASTER_VELOCITY_VOLTAGE_COMPENSATION_ENABLED, TuningConstants.POWERCELL_FLYWHEEL_MASTER_VELOCITY_VOLTAGE_COMPENSATION_MAXVOLTAGE);

            ITalonSRX flyWheelFollower = provider.getTalonSRX(ElectronicsConstants.POWERCELL_FLYWHEEL_FOLLOWER_CAN_ID);
            flyWheelFollower.setNeutralMode(MotorNeutralMode.Coast);
            flyWheelFollower.follow(this.flyWheel);
            flyWheelFollower.setInvertOutput(HardwareConstants.POWERCELL_FLYWHEEL_FOLLOWER_INVERT_OUTPUT);
            flyWheelFollower.setVoltageCompensation(TuningConstants.POWERCELL_FLYWHEEL_FOLLOWER_VELOCITY_VOLTAGE_COMPENSATION_ENABLED, TuningConstants.POWERCELL_FLYWHEEL_FOLLOWER_VELOCITY_VOLTAGE_COMPENSATION_MAXVOLTAGE);
        }
        else
        {
            this.flyWheel = null;
        }

        // carousel components:
        if (PowerCellMechanism.HAS_CAROUSEL_MOTOR)
        {
            this.carouselMotor = provider.getVictorSPX(ElectronicsConstants.POWERCELL_CAROUSEL_MOTOR_CAN_ID);
            this.carouselMotor.setInvertOutput(HardwareConstants.POWERCELL_CAROUSEL_MOTOR_INVERT_OUTPUT);
            this.carouselMotor.setControlMode(TalonSRXControlMode.PercentOutput);
            this.carouselMotor.setNeutralMode(MotorNeutralMode.Brake);
        }
        else
        {
            this.carouselMotor = null;
        }

        // kicker components:
        if (PowerCellMechanism.HAS_PNEUMATICS)
        {
            this.kickerSolenoid = provider.getDoubleSolenoid(ElectronicsConstants.PCM_A_MODULE, ElectronicsConstants.POWERCELL_KICKER_FORWARD_PCM, ElectronicsConstants.POWERCELL_KICKER_REVERSE_PCM);
        }
        else
        {
            this.kickerSolenoid = null;
        }

        if (PowerCellMechanism.HAS_SHOOTER_MOTOR)
        {
            this.kickerMotor = provider.getTalonSRX(ElectronicsConstants.POWERCELL_KICKER_MOTOR_CAN_ID);
            this.kickerMotor.setInvertOutput(HardwareConstants.POWERCELL_KICKER_MOTOR_INVERT_OUTPUT);
            this.kickerMotor.setControlMode(TalonSRXControlMode.Velocity);
            this.kickerMotor.setNeutralMode(MotorNeutralMode.Coast);
        }
        else
        {
            this.kickerMotor = null;
        }

        this.intakeExtended = false;
        this.flywheelVelocitySetpoint = 0.0;
    }

    @Override
    public void readSensors()
    {
        if (PowerCellMechanism.HAS_SHOOTER_MOTOR)
        {
            this.flywheelPosition = this.flyWheel.getPosition();
            this.flywheelVelocity = this.flyWheel.getVelocity();
            this.flywheelError = this.flyWheel.getError();
        }

        this.logger.logNumber(LoggingKey.PowerCellFlywheelVelocity, this.flywheelVelocity);
        this.logger.logNumber(LoggingKey.PowerCellFlywheelPosition, this.flywheelPosition);
        this.logger.logNumber(LoggingKey.PowerCellFlywheelError, this.flywheelError);
    }

    @Override
    public void update()
    {
        if (PowerCellMechanism.HAS_PNEUMATICS)
        {
            if (this.driver.getDigital(DigitalOperation.PowerCellHoodPointBlank))
            {
                this.innerHood.set(DoubleSolenoidValue.Reverse);
                this.outerHood.set(DoubleSolenoidValue.Reverse);
            }
            else if (this.driver.getDigital(DigitalOperation.PowerCellHoodShort))
            {
                this.innerHood.set(DoubleSolenoidValue.Forward);
                this.outerHood.set(DoubleSolenoidValue.Reverse);
            }
            else if (this.driver.getDigital(DigitalOperation.PowerCellHoodMedium))
            {
                this.innerHood.set(DoubleSolenoidValue.Reverse);
                this.outerHood.set(DoubleSolenoidValue.Forward);
            }
            else if (this.driver.getDigital(DigitalOperation.PowerCellHoodLong))
            {
                this.innerHood.set(DoubleSolenoidValue.Forward);
                this.outerHood.set(DoubleSolenoidValue.Forward);
            }
        }

        boolean kick = this.driver.getDigital(DigitalOperation.PowerCellKick);
        if (PowerCellMechanism.HAS_PNEUMATICS)
        {
            if (kick)
            {
                this.kickerSolenoid.set(DoubleSolenoidValue.Forward);
            }
            else
            {
                this.kickerSolenoid.set(DoubleSolenoidValue.Reverse);
            }
        }

        boolean kickerSpin = this.driver.getDigital(DigitalOperation.PowerCellKickerSpin);
        if (PowerCellMechanism.HAS_KICKER_MOTOR)
        {
            if (kickerSpin)
            {
                this.kickerMotor.set(TuningConstants.POWERCELL_KICKER_MOTOR_FEED_POWER);
            }
            else
            {
                this.kickerMotor.set(TuningConstants.PERRY_THE_PLATYPUS);
            }
        }

        boolean intakeExtend = this.driver.getDigital(DigitalOperation.PowerCellIntakeExtend);
        boolean intakeRetract = !intakeExtend && this.driver.getDigital(DigitalOperation.PowerCellIntakeRetract);
        if (PowerCellMechanism.HAS_PNEUMATICS)
        {
            if (intakeExtend)
            {
                this.intakeExtended = true;
                this.intakeSolenoid.set(DoubleSolenoidValue.Forward);
            }
            else if (intakeRetract)
            {
                this.intakeExtended = false;
                this.intakeSolenoid.set(DoubleSolenoidValue.Reverse);
            }
        }

        boolean isIntaking = this.driver.getDigital(DigitalOperation.PowerCellIntake);
        if (PowerCellMechanism.HAS_INTAKE_MOTOR)
        {
            if (isIntaking)
            {
                this.rollerMotor.set(TuningConstants.POWERCELL_ROLLER_MOTOR_INTAKE_POWER);
            }
            else if (this.driver.getDigital(DigitalOperation.PowerCellOuttake))
            {
                this.rollerMotor.set(TuningConstants.POWERCELL_ROLLER_MOTOR_OUTTAKE_POWER);
            }
            else
            {
                this.rollerMotor.set(TuningConstants.PERRY_THE_PLATYPUS);
            }
        }

        double flywheelVelocityPercentage = this.driver.getAnalog(AnalogOperation.PowerCellFlywheelVelocity);
        if (flywheelVelocityPercentage != TuningConstants.MAGIC_NULL_VALUE)
        {
           if (Math.abs(flywheelVelocityPercentage) < 0.01)
           {
               // instead of trying to ensure the wheel is going at a speed of 0, let's just disable the motor
               this.flywheelVelocitySetpoint = 0.0;
           }
           else
           {
               this.flywheelVelocitySetpoint = flywheelVelocityPercentage * TuningConstants.POWERCELL_FLYWHEEL_ONE_VELOCITY_PID_KS;
           }
        }

        if (PowerCellMechanism.HAS_SHOOTER_MOTOR)
        {
            if (this.flywheelVelocitySetpoint == 0.0)
            {
                this.flyWheel.stop();
            }
            else
            {
                this.flyWheel.set(this.flywheelVelocitySetpoint);
            }
        }

        this.logger.logNumber(LoggingKey.PowerCellFlywheelVelocitySetpoint, this.flywheelVelocitySetpoint);

        double desiredCarouselMotorPower = TuningConstants.PERRY_THE_PLATYPUS;
        double debugCarouselMotorPower = this.driver.getAnalog(AnalogOperation.PowerCellCarousel);
        if (debugCarouselMotorPower != TuningConstants.MAGIC_NULL_VALUE)
        {
            desiredCarouselMotorPower = debugCarouselMotorPower;
        }
        else if (this.driver.getDigital(DigitalOperation.PowerCellRotateCarousel) && kickerSpin && this.flywheelVelocitySetpoint != 0.0) 
        {
            desiredCarouselMotorPower = TuningConstants.POWERCELL_CAROUSEL_MOTOR_POWER_SHOOTING;
        }
        else if (isIntaking)
        {
            desiredCarouselMotorPower = TuningConstants.POWERCELL_CAROUSEL_MOTOR_POWER_INDEXING;
        }

        if (PowerCellMechanism.HAS_CAROUSEL_MOTOR)
        {
            this.carouselMotor.set(desiredCarouselMotorPower);
        }

        this.logger.logNumber(LoggingKey.PowerCellCarouselPower, desiredCarouselMotorPower);
        this.logger.logBoolean(LoggingKey.PowerCellIsIntaking, isIntaking);
        this.logger.logBoolean(LoggingKey.PowerCellIntakeExtended, this.intakeExtended);
    }

    @Override
    public void stop()
    {
        if (PowerCellMechanism.HAS_PNEUMATICS)
        {
            this.intakeSolenoid.set(DoubleSolenoidValue.Off);
        }

        if (PowerCellMechanism.HAS_INTAKE_MOTOR)
        {
            this.rollerMotor.stop();
        }

        if (PowerCellMechanism.HAS_PNEUMATICS)
        {
            this.innerHood.set(DoubleSolenoidValue.Off);
            this.outerHood.set(DoubleSolenoidValue.Off);
        }

        if (PowerCellMechanism.HAS_SHOOTER_MOTOR)
        {
            this.flyWheel.stop();
        }

        if (PowerCellMechanism.HAS_CAROUSEL_MOTOR)
        {
            this.carouselMotor.stop();
        }

        if (PowerCellMechanism.HAS_PNEUMATICS)
        {
            this.kickerSolenoid.set(DoubleSolenoidValue.Off);
        }

        if (PowerCellMechanism.HAS_KICKER_MOTOR)
        {
            this.kickerMotor.stop();
        }
    }

    public double getFlywheelVelocity()
    {
        return this.flywheelVelocity;
    }

    public double getFlywheelVelocitySetpoint()
    {
        return this.flywheelVelocitySetpoint;
    }

    public boolean isFlywheelSpunUp()
    {
        return this.flywheelVelocitySetpoint > 0.0 && Math.abs(this.flywheelError) <= TuningConstants.POWERCELL_FLYWHEEL_ALLOWABLE_ERROR_RANGE;
    }
}