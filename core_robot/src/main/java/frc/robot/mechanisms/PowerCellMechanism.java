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
    private final ITimer timer;

    private static final int slotId = 0;

    private final IDriver driver;
    private final ILogger logger;

    private final IDoubleSolenoid intakeSolenoid;
    private final ISparkMax rollerMotor;

    private final IDoubleSolenoid outerHood;
    private final IDoubleSolenoid innerHood;
    private final ITalonSRX flyWheel;

    private final ITalonSRX carouselMotor;
    private final IEncoder carouselEncoder;

    private final IDoubleSolenoid kickerSolenoid;
    private final IVictorSPX kickerMotor;

    private double flywheelPosition;
    private double flywheelVelocity;
    private double flywheelError;

    private double carouselPosition;
    private double carouselVelocity;

    private PIDHandler carouselPID;

    private boolean intakeExtended;

    private double flywheelVelocitySetpoint;

    @Inject
    public PowerCellMechanism(IDriver driver, LoggingManager logger, ITimer timer, IRobotProvider provider)
    {
        this.driver = driver;
        this.logger = logger;
        this.timer = timer;

        // intake components:
        this.intakeSolenoid = provider.getDoubleSolenoid(ElectronicsConstants.PNEUMATICS_MODULE_A, ElectronicsConstants.PNEUMATICS_MODULE_TYPE_A, ElectronicsConstants.POWERCELL_INTAKE_FORWARD_PCM, ElectronicsConstants.POWERCELL_INTAKE_REVERSE_PCM);
        this.rollerMotor = provider.getSparkMax(ElectronicsConstants.POWERCELL_ROLLER_MOTOR_CAN_ID, SparkMaxMotorType.Brushless);
        this.rollerMotor.setInvertOutput(HardwareConstants.POWERCELL_ROLLER_MOTOR_INVERT_OUTPUT);
        this.rollerMotor.setControlMode(SparkMaxControlMode.PercentOutput);
        this.rollerMotor.setNeutralMode(MotorNeutralMode.Brake);

        // shooter components:
        this.outerHood = provider.getDoubleSolenoid(ElectronicsConstants.PNEUMATICS_MODULE_A, ElectronicsConstants.PNEUMATICS_MODULE_TYPE_A, ElectronicsConstants.POWERCELL_OUTER_HOOD_FORWARD_PCM, ElectronicsConstants.POWERCELL_OUTER_HOOD_REVERSE_PCM);
        this.innerHood = provider.getDoubleSolenoid(ElectronicsConstants.PNEUMATICS_MODULE_A, ElectronicsConstants.PNEUMATICS_MODULE_TYPE_A, ElectronicsConstants.POWERCELL_INNER_HOOD_FORWARD_PCM, ElectronicsConstants.POWERCELL_INNER_HOOD_REVERSE_PCM);
        this.flyWheel = provider.getTalonSRX(ElectronicsConstants.POWERCELL_FLYWHEEL_MASTER_CAN_ID);
        this.flyWheel.setInvertOutput(HardwareConstants.POWERCELL_FLYWHEEL_MASTER_INVERT_OUTPUT);
        this.flyWheel.setInvertSensor(HardwareConstants.POWERCELL_FLYWHEEL_MASTER_INVERT_SENSOR);
        this.flyWheel.setNeutralMode(MotorNeutralMode.Coast);
        this.flyWheel.setSensorType(TalonXFeedbackDevice.QuadEncoder);
        this.flyWheel.setPosition(0);
        this.flyWheel.setControlMode(TalonXControlMode.Velocity);
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

        // carousel components:
        this.carouselMotor = provider.getTalonSRX(ElectronicsConstants.POWERCELL_CAROUSEL_MOTOR_CAN_ID);
        this.carouselMotor.setInvertOutput(HardwareConstants.POWERCELL_CAROUSEL_MOTOR_INVERT_OUTPUT);
        this.carouselMotor.setControlMode(TalonXControlMode.PercentOutput);
        this.carouselMotor.setNeutralMode(MotorNeutralMode.Brake);

        this.carouselEncoder = provider.getEncoder(ElectronicsConstants.POWERCELL_CAROUSEL_ENCODER_CHANNEL_A, ElectronicsConstants.POWERCELL_CAROUSEL_ENCODER_CHANNEL_B);
        this.carouselPID = new PIDHandler(
            TuningConstants.POWERCELL_CAROUSEL_PID_KP,
            TuningConstants.POWERCELL_CAROUSEL_PID_KI,
            TuningConstants.POWERCELL_CAROUSEL_PID_KD,
            TuningConstants.POWERCELL_CAROUSEL_PID_KF,
            TuningConstants.POWERCELL_CAROUSEL_PID_KS,
            -TuningConstants.POWERCELL_CAROUSEL_MAX_POWER,
            TuningConstants.POWERCELL_CAROUSEL_MAX_POWER,
            this.timer);

        // kicker components:
        this.kickerSolenoid = provider.getDoubleSolenoid(ElectronicsConstants.PNEUMATICS_MODULE_A, ElectronicsConstants.PNEUMATICS_MODULE_TYPE_A, ElectronicsConstants.POWERCELL_KICKER_FORWARD_PCM, ElectronicsConstants.POWERCELL_KICKER_REVERSE_PCM);
        this.kickerMotor = provider.getVictorSPX(ElectronicsConstants.POWERCELL_KICKER_MOTOR_CAN_ID);
        this.kickerMotor.setInvertOutput(HardwareConstants.POWERCELL_KICKER_MOTOR_INVERT_OUTPUT);
        this.kickerMotor.setControlMode(TalonXControlMode.Velocity);
        this.kickerMotor.setNeutralMode(MotorNeutralMode.Coast);

        this.intakeExtended = false;
        this.flywheelVelocitySetpoint = 0.0;
    }

    @Override
    public void readSensors()
    {
        this.flywheelPosition = this.flyWheel.getPosition();
        this.flywheelVelocity = this.flyWheel.getVelocity();
        this.flywheelError = this.flyWheel.getError();

        this.carouselPosition = this.carouselEncoder.getDistance();
        this.carouselVelocity = this.carouselEncoder.getRate();

        this.logger.logNumber(LoggingKey.PowerCellFlywheelVelocity, this.flywheelVelocity);
        this.logger.logNumber(LoggingKey.PowerCellFlywheelPosition, this.flywheelPosition);
        this.logger.logNumber(LoggingKey.PowerCellFlywheelError, this.flywheelError);

        this.logger.logNumber(LoggingKey.PowerCellCarouselVelocity, this.carouselVelocity);
        this.logger.logNumber(LoggingKey.PowerCellCarouselPosition, this.carouselPosition);
    }

    @Override
    public void update()
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

        boolean kick = this.driver.getDigital(DigitalOperation.PowerCellKick);
        if (kick)
        {
            this.kickerSolenoid.set(DoubleSolenoidValue.Forward);
        }
        else
        {
            this.kickerSolenoid.set(DoubleSolenoidValue.Reverse);
        }

        boolean kickerSpin = this.driver.getDigital(DigitalOperation.PowerCellKickerSpin);
        if (kickerSpin)
        {
            this.kickerMotor.set(TuningConstants.POWERCELL_KICKER_MOTOR_FEED_POWER);
        }
        else if (this.driver.getDigital(DigitalOperation.PowerCellKickerSpinReverse))
        {
            this.kickerMotor.set(TuningConstants.POWERCELL_KICKER_MOTOR_FEED_REVERSE_POWER);
        }
        else
        {
            this.kickerMotor.set(TuningConstants.PERRY_THE_PLATYPUS);
        }

        boolean intakeExtend = this.driver.getDigital(DigitalOperation.PowerCellIntakeExtend);
        boolean intakeRetract = !intakeExtend && this.driver.getDigital(DigitalOperation.PowerCellIntakeRetract);
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

        boolean isIntaking = this.driver.getDigital(DigitalOperation.PowerCellIntake);
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

        double flywheelVelocityPercentage = this.driver.getAnalog(AnalogOperation.PowerCellFlywheelVelocity) + this.driver.getAnalog(AnalogOperation.PowerCellFlywheelVelocity2);
        if (flywheelVelocityPercentage != TuningConstants.MAGIC_NULL_VALUE)
        {
           if (Math.abs(flywheelVelocityPercentage) < 0.01)
           {
               // instead of trying to ensure the wheel is going at a speed of 0, let's just disable the motor
               this.flywheelVelocitySetpoint = 0.0;
           }
           else
           {
               this.flywheelVelocitySetpoint = ((flywheelVelocityPercentage + TuningConstants.POWERCELL_FLYWHEEL_MIN_PERCENTILE) * TuningConstants.POWERCELL_FLYWHEEL_ONE_VELOCITY_PID_KS);
           }
        }

        if (this.flywheelVelocitySetpoint == 0.0)
        {
            this.flyWheel.stop();
        }
        else if (this.driver.getDigital(DigitalOperation.PowerCellFlywheelReverse))
        {
            this.flyWheel.setControlMode(TalonXControlMode.PercentOutput);
            this.flyWheel.set(TuningConstants.POWERCELL_FLYWHEEL_REVERSE_POWER);
        }
        else
        {
            this.flyWheel.setControlMode(TalonXControlMode.Velocity);
            this.flyWheel.set(this.flywheelVelocitySetpoint);
        }

        this.logger.logNumber(LoggingKey.PowerCellFlywheelVelocitySetpoint, this.flywheelVelocitySetpoint);

        double desiredCarouselVelocity = TuningConstants.PERRY_THE_PLATYPUS;
        double debugCarouselVelocity = this.driver.getAnalog(AnalogOperation.PowerCellCarousel);
        if (debugCarouselVelocity != TuningConstants.PERRY_THE_PLATYPUS)
        {
            desiredCarouselVelocity = 1.0 * debugCarouselVelocity * 0.5;
        }
        else if (this.driver.getDigital(DigitalOperation.PowerCellRotateCarousel) && kickerSpin && this.flywheelVelocitySetpoint != 0.0) 
        {
            desiredCarouselVelocity = TuningConstants.POWERCELL_CAROUSEL_MOTOR_POWER_SHOOTING;
        }
        else if (isIntaking)
        {
            desiredCarouselVelocity = TuningConstants.POWERCELL_CAROUSEL_MOTOR_POWER_INDEXING;
        }

        double desiredCarouselMotorPower;
        if (TuningConstants.POWERCELL_CAROUSEL_USE_PID)
        {
            desiredCarouselMotorPower = this.carouselPID.calculateVelocity(desiredCarouselVelocity, this.carouselVelocity);
        }
        else
        {
            desiredCarouselMotorPower = desiredCarouselVelocity;
        }

        this.carouselMotor.set(desiredCarouselMotorPower);
        
        this.logger.logNumber(LoggingKey.PowerCellCarouselPower, desiredCarouselMotorPower);
        this.logger.logNumber(LoggingKey.PowerCellDesiredCarouselVelocity, desiredCarouselVelocity);
        this.logger.logBoolean(LoggingKey.PowerCellIsIntaking, isIntaking);
        this.logger.logBoolean(LoggingKey.PowerCellIntakeExtended, this.intakeExtended);
    }

    @Override
    public void stop()
    {
        this.intakeSolenoid.set(DoubleSolenoidValue.Off);
        this.rollerMotor.stop();

        this.innerHood.set(DoubleSolenoidValue.Off);
        this.outerHood.set(DoubleSolenoidValue.Off);
        this.flyWheel.stop();

        this.carouselPID.reset();
        this.carouselMotor.stop();

        this.kickerSolenoid.set(DoubleSolenoidValue.Off);
        this.kickerMotor.stop();
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

    public double getCarouselVelocity()
    {
        return this.carouselVelocity;
    }

    public double getCarouselPosition()
    {
        return this.carouselPosition;
    }
}
