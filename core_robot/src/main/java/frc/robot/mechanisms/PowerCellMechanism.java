package frc.robot.mechanisms;

import frc.robot.*;
import frc.robot.common.*;
import frc.robot.common.robotprovider.*;
import frc.robot.driver.*;
import frc.robot.driver.common.*;

import com.google.inject.Inject;
import com.google.inject.Singleton;

@Singleton
public class PowerCellMechanism implements IMechanism
{
    //private static final int slotId = 0;

    private final ILogger logger;

    private final IDoubleSolenoid intakeSolenoid;
    private final ISparkMax rollerMotor;
    //private final ISparkMax rollerMotorOuter;

    //private final IDoubleSolenoid outerHood;
    //private final IDoubleSolenoid innerHood;
    //private final IDoubleSolenoid kickerSolenoid;
    //private final ITalonSRX flyWheel;
    //private final ITalonSRX turret;

    //private final IVictorSPX genevaMotor;
    //private final ITimer timer;

    //private final ICounter carouselCounter;
    //private final IAnalogInput throughBeamSensor;

    private Driver driver;

    //private double flywheelPosition;
    //private double flywheelVelocity;
    //private double flywheelError;
    //private double turretPosition;
    //private double turretVelocity;
    //private double turretError;
    //private int carouselCount;

    //private double startingTurretOffsetAngle;

    //private boolean[] hasPowerCell;
    //private int currentCarouselIndex;
    //private double lastCarouselCountTime;

    private boolean intakeExtended;

    //private CarouselState carouselState;
    //private int previousIndex;
    //private double lastIntakeTime;
    //private double flywheelVelocitySetpoint;

    @Inject
    public PowerCellMechanism(LoggingManager logger, IRobotProvider provider)
    {
        this.logger = logger;

        //this.throughBeamSensor = provider.getAnalogInput(ElectronicsConstants.POWERCELL_THROUGHBEAM_ANALOG_INPUT);

        this.intakeSolenoid = provider.getDoubleSolenoid(ElectronicsConstants.PCM_A_MODULE, ElectronicsConstants.POWERCELL_INTAKE_FORWARD_PCM, ElectronicsConstants.POWERCELL_INTAKE_REVERSE_PCM);
        //this.kickerSolenoid = provider.getDoubleSolenoid(ElectronicsConstants.PCM_A_MODULE, ElectronicsConstants.POWERCELL_KICKER_FORWARD_PCM, ElectronicsConstants.POWERCELL_KICKER_REVERSE_PCM);
        //this.outerHood = provider.getDoubleSolenoid(ElectronicsConstants.PCM_B_MODULE, ElectronicsConstants.POWERCELL_OUTER_HOOD_FORWARD_PCM, ElectronicsConstants.POWERCELL_OUTER_HOOD_REVERSE_PCM);
        //this.innerHood = provider.getDoubleSolenoid(ElectronicsConstants.PCM_B_MODULE, ElectronicsConstants.POWERCELL_INNER_HOOD_FORWARD_PCM, ElectronicsConstants.POWERCELL_INNER_HOOD_REVERSE_PCM);

        this.rollerMotor = provider.getSparkMax(ElectronicsConstants.POWERCELL_ROLLER_MOTOR_CAN_ID, SparkMaxMotorType.Brushless);
        this.rollerMotor.setInvertOutput(HardwareConstants.POWERCELL_ROLLER_MOTOR_INVERT_OUTPUT);
        this.rollerMotor.setControlMode(SparkMaxControlMode.PercentOutput);
        this.rollerMotor.setNeutralMode(MotorNeutralMode.Brake);

        //this.rollerMotorOuter = provider.getSparkMax(ElectronicsConstants.POWERCELL_OUTER_ROLLER_MOTOR_CAN_ID, SparkMaxMotorType.Brushless);
        //this.rollerMotorOuter.setInvertOutput(HardwareConstants.POWERCELL_ROLLER_MOTOR_OUTER_INVERT_OUTPUT);
        //this.rollerMotorOuter.setControlMode(SparkMaxControlMode.PercentOutput);
        //this.rollerMotorOuter.setNeutralMode(MotorNeutralMode.Brake);

        //this.flyWheel = provider.getTalonSRX(ElectronicsConstants.POWERCELL_FLYWHEEL_MASTER_CAN_ID);
        //this.flyWheel.setInvertOutput(HardwareConstants.POWERCELL_FLYWHEEL_MASTER_INVERT_OUTPUT);
        //this.flyWheel.setInvertSensor(HardwareConstants.POWERCELL_FLYWHEEL_MASTER_INVERT_SENSOR);
        //this.flyWheel.setNeutralMode(MotorNeutralMode.Coast);
        //this.flyWheel.setSensorType(TalonXFeedbackDevice.QuadEncoder);
        //this.flyWheel.setPosition(0);
        //this.flyWheel.setControlMode(TalonSRXControlMode.Velocity);
        //this.flyWheel.setPIDF(
        //    TuningConstants.POWERCELL_FLYWHEEL_ONE_VELOCITY_PID_KP,
        //    TuningConstants.POWERCELL_FLYWHEEL_ONE_VELOCITY_PID_KI,
        //    TuningConstants.POWERCELL_FLYWHEEL_ONE_VELOCITY_PID_KD,
        //    TuningConstants.POWERCELL_FLYWHEEL_ONE_VELOCITY_PID_KF,
        //    PowerCellMechanism.slotId);
        //this.flyWheel.configureVelocityMeasurements(TuningConstants.POWERCELL_FLYWHEEL_VELOCITY_PERIOD, TuningConstants.POWERCELL_FLYWHEEL_VELOCITY_WINDOWSIZE);
        //this.flyWheel.setVoltageCompensation(TuningConstants.POWERCELL_FLYWHEEL_MASTER_VELOCITY_VOLTAGE_COMPENSATION_ENABLED, TuningConstants.POWERCELL_FLYWHEEL_MASTER_VELOCITY_VOLTAGE_COMPENSATION_MAXVOLTAGE);

        //ITalonSRX flyWheelFollower = provider.getTalonSRX(ElectronicsConstants.POWERCELL_FLYWHEEL_FOLLOWER_CAN_ID);
        //flyWheelFollower.setNeutralMode(MotorNeutralMode.Coast);
        //flyWheelFollower.follow(this.flyWheel);
        //flyWheelFollower.setInvertOutput(HardwareConstants.POWERCELL_FLYWHEEL_FOLLOWER_INVERT_OUTPUT);
        //flyWheelFollower.setVoltageCompensation(TuningConstants.POWERCELL_FLYWHEEL_FOLLOWER_VELOCITY_VOLTAGE_COMPENSATION_ENABLED, TuningConstants.POWERCELL_FLYWHEEL_FOLLOWER_VELOCITY_VOLTAGE_COMPENSATION_MAXVOLTAGE);

        //this.turret = provider.getTalonSRX(ElectronicsConstants.POWERCELL_TURRET_MOTOR_CAN_ID);
        //this.turret.setInvertOutput(HardwareConstants.POWERCELL_TURRET_INVERT_OUTPUT);
        //this.turret.setInvertSensor(HardwareConstants.POWERCELL_TURRET_INVERT_SENSOR);
        //this.turret.setNeutralMode(MotorNeutralMode.Brake);
        //this.turret.setSensorType(TalonXFeedbackDevice.QuadEncoder);
        //this.turret.setPosition(0);
        //this.turret.setForwardLimitSwitch(TuningConstants.POWERCELL_TURRET_FORWARD_LIMIT_SWITCH_ENABLED, TuningConstants.POWERCELL_TURRET_FORWARD_LIMIT_SWITCH_NORMALLY_OPEN);
        //this.turret.setReverseLimitSwitch(TuningConstants.POWERCELL_TURRET_REVERSE_LIMIT_SWITCH_ENABLED, TuningConstants.POWERCELL_TURRET_REVERSE_LIMIT_SWITCH_NORMALLY_OPEN);
        //if (TuningConstants.POWERCELL_TURRET_USE_PID)
       // {
        //    this.turret.setControlMode(TalonSRXControlMode.Position);
        //    this.turret.setPIDF(
        //        TuningConstants.POWERCELL_TURRET_POSITION_PID_KP,
        //        TuningConstants.POWERCELL_TURRET_POSITION_PID_KI,
        //        TuningConstants.POWERCELL_TURRET_POSITION_PID_KD,
        //        TuningConstants.POWERCELL_TURRET_POSITION_PID_KF,
        //        PowerCellMechanism.slotId);
        //}
        //else
        //{
        //    this.turret.setControlMode(TalonSRXControlMode.PercentOutput);
        //}

        //this.genevaMotor = provider.getVictorSPX(ElectronicsConstants.POWERCELL_GENEVA_MOTOR_CAN_ID);
        //this.genevaMotor.setInvertOutput(HardwareConstants.POWERCELL_GENEVA_MOTOR_INVERT_OUTPUT);
        //this.genevaMotor.setControlMode(TalonSRXControlMode.PercentOutput);
        //this.genevaMotor.setNeutralMode(MotorNeutralMode.Brake);

        //this.timer = timer;

        //this.carouselCounter = provider.getCounter(ElectronicsConstants.POWERCELL_CAROUSEL_COUNTER_DIO);

        //this.carouselState = CarouselState.Stationary;
        //this.previousIndex = 0;
        //this.lastIntakeTime = -2.0;
        //this.lastCarouselCountTime = -2.0;

        //this.hasPowerCell = new boolean[5];
        //this.currentCarouselIndex = 0;

        this.intakeExtended = false;
        //this.flywheelVelocitySetpoint = 0.0;
    }

    @Override
    public void readSensors()
    {
        //double currentTime = this.timer.get();

        //boolean throughBeamBroken = false;
        //double throughBeamVoltage = this.throughBeamSensor.getVoltage();
        //if (throughBeamVoltage < TuningConstants.POWERCELL_TROUGHBEAM_CUTOFF)
        //{
        //    throughBeamBroken = true;
        //}

        //this.turretPosition = this.turret.getPosition();
        //this.turretVelocity = this.turret.getVelocity();
        //this.turretError = this.turret.getError();

        //this.flywheelPosition = this.flyWheel.getPosition();
        //this.flywheelVelocity = this.flyWheel.getVelocity();
        //this.flywheelError = this.flyWheel.getError();

        //int newCarouselCount = this.carouselCounter.get();
        //if (newCarouselCount > this.carouselCount &&
        //    currentTime - this.lastCarouselCountTime > TuningConstants.POWERCELL_GENEVA_COUNT_THRESHOLD)
        //{
        //    int slotsToAdvance = 1;
        //    if (this.carouselState == CarouselState.MovingToPrevious)
         //   {
        //        // use (#slots - 1) instead of just (-1) to keep the modulo positive
        //        slotsToAdvance = HardwareConstants.POWERCELL_CAROUSEL_SLOT_COUNT - 1;
        //    }

        //    this.currentCarouselIndex = (currentCarouselIndex + slotsToAdvance) % HardwareConstants.POWERCELL_CAROUSEL_SLOT_COUNT;
        //    this.lastCarouselCountTime = currentTime;

            // only register whether throughbeam is broken when we are switching to a new slot
        //    this.hasPowerCell[this.currentCarouselIndex] = throughBeamBroken;
        //}

        //this.carouselCount = newCarouselCount;

        // this.logger.logNumber(LoggingKey.PowerCellTurretVelocity, this.turretVelocity);
        // this.logger.logNumber(LoggingKey.PowerCellTurretPosition, this.turretPosition);
        // this.logger.logNumber(LoggingKey.PowerCellTurretError, this.turretError);
        // this.logger.logNumber(LoggingKey.PowerCellFlywheelVelocity, this.flywheelVelocity);
        // this.logger.logNumber(LoggingKey.PowerCellFlywheelPosition, this.flywheelPosition);
        // this.logger.logNumber(LoggingKey.PowerCellFlywheelError, this.flywheelError);
        // this.logger.logInteger(LoggingKey.PowerCellCarouselCount, this.carouselCount);
        // this.logger.logInteger(LoggingKey.PowerCellCarouselCurrentIndex, this.currentCarouselIndex);
        // this.logger.logNumber(LoggingKey.PowerCellThroughBeamVoltage, throughBeamVoltage);
        // this.logger.logBoolean(LoggingKey.PowerCellThroughBeamBroken, throughBeamBroken);
        // this.logger.logBooleanArray(LoggingKey.PowerCellHasPowerCell, this.hasPowerCell);
    }

    @Override
    public void update()
    {
        //double currentTime = this.timer.get();

        //double startingTurretOffset = this.driver.getAnalog(AnalogOperation.PowerCellTurretOffset);
        //if (startingTurretOffset != 0.0)
        //{
        //    this.startingTurretOffsetAngle = startingTurretOffset;
        //}

        //if (this.driver.getDigital(DigitalOperation.PowerCellResetTurretFront))
        //{
        //    this.startingTurretOffsetAngle = 0.0;
        //    this.turret.setPosition(0);
        //    this.turret.set(0.0);
        //}

        //if (this.driver.getDigital(DigitalOperation.PowerCellHoodPointBlank))
        //{
        //    this.innerHood.set(DoubleSolenoidValue.Reverse);
        //    this.outerHood.set(DoubleSolenoidValue.Reverse);
        //}
        //else if (this.driver.getDigital(DigitalOperation.PowerCellHoodShort))
        //{
        //    this.innerHood.set(DoubleSolenoidValue.Forward);
        //    this.outerHood.set(DoubleSolenoidValue.Reverse);
        //}
        //else if (this.driver.getDigital(DigitalOperation.PowerCellHoodMedium))
        //{
        //    this.innerHood.set(DoubleSolenoidValue.Reverse);
        //    this.outerHood.set(DoubleSolenoidValue.Forward);
        //}
        //else if (this.driver.getDigital(DigitalOperation.PowerCellHoodLong))
        //{
        //    this.innerHood.set(DoubleSolenoidValue.Forward);
       //     this.outerHood.set(DoubleSolenoidValue.Forward);
        //}

        //boolean kick = this.driver.getDigital(DigitalOperation.PowerCellKick);
        //if (kick)
        //{
        //    this.hasPowerCell[this.currentCarouselIndex] = false;
        //    this.kickerSolenoid.set(DoubleSolenoidValue.Forward);
        //}
        //else
        //{
        //    this.kickerSolenoid.set(DoubleSolenoidValue.Reverse);
        //}

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
            this.rollerMotor.set(TuningConstants.STHOPE_BLEASE);
        }

        //double flywheelVelocityPercentage = this.driver.getAnalog(AnalogOperation.PowerCellFlywheelVelocity);
        //if (flywheelVelocityPercentage != TuningConstants.MAGIC_NULL_VALUE)
        //{
        //    if (Math.abs(flywheelVelocityPercentage) < 0.01)
        //    {
        //        // instead of trying to ensure the wheel is going at a speed of 0, let's just disable the motor
        //        this.flywheelVelocitySetpoint = 0.0;
        //    }
        //    else
         //   {
        //        this.flywheelVelocitySetpoint = flywheelVelocityPercentage * TuningConstants.POWERCELL_FLYWHEEL_ONE_VELOCITY_PID_KS;
        //    }
        //}

        // if (this.flywheelVelocitySetpoint == 0.0)
        // {
        //     this.flyWheel.stop();
        // }
        // else
        // {
        //     this.flyWheel.set(this.flywheelVelocitySetpoint);
        // }

        // this.logger.logNumber(LoggingKey.PowerCellFlywheelVelocitySetpoint, this.flywheelVelocitySetpoint);

        // double desiredTurretPosition = this.driver.getAnalog(AnalogOperation.PowerCellTurretPosition);
        // this.logger.logNumber(LoggingKey.PowerCellTurretPositionDesired, desiredTurretPosition);
        // if (TuningConstants.POWERCELL_TURRET_USE_PID)
        // {
        //     if (desiredTurretPosition != TuningConstants.POWERCELL_TURRET_MAGIC_DONT_MOVE_VALUE)
        //     {
        //         desiredTurretPosition = this.getClosestAngleInRange(desiredTurretPosition, this.getTurretPosition(), HardwareConstants.POWERCELL_TURRET_MINIMUM_RANGE_VALUE, HardwareConstants.POWERCELL_TURRET_MAXIMUM_RANGE_VALUE);
        //         this.turret.set((desiredTurretPosition + startingTurretOffsetAngle) * HardwareConstants.POWERCELL_TURRET_DEGREES_TO_TICKS);
        //     }
        // }
        // else
        // {
        //     this.turret.set(desiredTurretPosition);
        // }

        // // determine if we should make any transition from our current carousel state:
        // switch (this.carouselState)
        // {
        //     case Stationary:
        //         if (!this.intakeExtended || kick)
        //         {
        //             // don't change state if the intake is retracted
        //         }
        //         else if (this.driver.getDigital(DigitalOperation.PowerCellMoveToNextSlot))
        //         {
        //             this.lastCarouselCountTime = currentTime;
        //             this.previousIndex = this.currentCarouselIndex;
        //             this.carouselState = CarouselState.MovingToNext;
        //         }
        //         else if (this.driver.getDigital(DigitalOperation.PowerCellMoveToPreviousSlot))
        //         {
        //             this.lastCarouselCountTime = currentTime;
        //             this.previousIndex = this.currentCarouselIndex;
        //             this.carouselState = CarouselState.MovingToPrevious;
        //         }
        //         // else if (isIntaking)
        //         // {
        //         //     // if intaking, start automatically indexing
        //         //     this.lastCarouselCountTime = currentTime;
        //         //     this.carouselState = CarouselState.Indexing;
        //         //     this.lastIntakeTime = currentTime;
        //         // }

        //         break;

        //     case Indexing:
        //         if (!this.intakeExtended || kick)
        //         {
        //             // become stationary if intake is retracted, or we're kicking
        //             this.carouselState = CarouselState.Stationary;
        //         }
        //         else if (this.driver.getDigital(DigitalOperation.PowerCellMoveToNextSlot))
        //         {
        //             this.previousIndex = this.currentCarouselIndex;
        //             this.carouselState = CarouselState.MovingToNext;
        //         }
        //         else if (this.driver.getDigital(DigitalOperation.PowerCellMoveToPreviousSlot))
        //         {
        //             this.previousIndex = this.currentCarouselIndex;
        //             this.carouselState = CarouselState.MovingToPrevious;
        //         }
        //         else if (isIntaking)
        //         {
        //             // if intaking, keep track of time
        //             this.lastIntakeTime = currentTime;
        //         }
        //         else if (currentTime - this.lastIntakeTime > TuningConstants.POWERCELL_GENEVA_MECHANISM_INDEXING_TIMEOUT)
        //         {
        //             // switch to move-to-next to avoid stopping when we are not fully turned
        //             this.previousIndex = this.currentCarouselIndex;
        //             this.carouselState = CarouselState.MovingToNext;
        //         }

        //         break;

        //     case MovingToNext:
        //         if (!this.intakeExtended || kick)
        //         {
        //             // become stationary if intake is retracted, or we're kicking
        //             this.carouselState = CarouselState.Stationary;
        //         }
        //         else if (this.driver.getDigital(DigitalOperation.PowerCellMoveToPreviousSlot))
        //         {
        //             this.previousIndex = this.currentCarouselIndex + 1;
        //             this.carouselState = CarouselState.MovingToPrevious;
        //         }
        //         else if (this.currentCarouselIndex != this.previousIndex)
        //         {
        //             this.carouselState = CarouselState.Stationary;
        //         }

        //         break;

        //     case MovingToPrevious:
        //         if (!this.intakeExtended || kick)
        //         {
        //             // become stationary if intake is retracted, or we're kicking
        //             this.carouselState = CarouselState.Stationary;
        //         }
        //         else if (this.driver.getDigital(DigitalOperation.PowerCellMoveToNextSlot))
        //         {
        //             this.previousIndex = this.currentCarouselIndex - 1;
        //             this.carouselState = CarouselState.MovingToNext;
        //         }
        //         else if (this.currentCarouselIndex != this.previousIndex)
        //         {
        //             this.carouselState = CarouselState.Stationary;
        //         }

        //         break;
        // }

        // // perform what we should do based on our current hopper state, or debug override:
        // double desiredGenevaMotorPower = TuningConstants.STHOPE_BLEASE;
        // double debugGenevaMotorPower = this.driver.getAnalog(AnalogOperation.PowerCellGenevaPower);
        // if (debugGenevaMotorPower != TuningConstants.MAGIC_NULL_VALUE)
        // {
        //     this.carouselState = CarouselState.Stationary;
        //     desiredGenevaMotorPower = debugGenevaMotorPower;
        // }
        // else
        // {
        //     switch (this.carouselState)
        //     {
        //         case Indexing:
        //             desiredGenevaMotorPower = TuningConstants.POWERCELL_GENEVA_MECHANISM_MOTOR_POWER_INDEXING;
        //             break;

        //         case MovingToNext:
        //             if (this.flywheelVelocitySetpoint != 0.0)
        //             {
        //                 desiredGenevaMotorPower = TuningConstants.POWERCELL_GENEVA_MECHANISM_MOTOR_POWER_SHOOTING;
        //             }
        //             else
        //             {
        //                 desiredGenevaMotorPower = TuningConstants.POWERCELL_GENEVA_MECHANISM_MOTOR_POWER_INDEXING;
        //             }

        //             break;

        //         case MovingToPrevious:
        //             desiredGenevaMotorPower = -TuningConstants.POWERCELL_GENEVA_MECHANISM_MOTOR_POWER_INDEXING;
        //             break;

        //         case Stationary:
        //             desiredGenevaMotorPower = TuningConstants.STHOPE_BLEASE;
        //             break;
        //     }
        // }

        // this.genevaMotor.set(desiredGenevaMotorPower);

        //this.logger.logString(LoggingKey.PowerCellCarouselState, this.carouselState.toString());
        //this.logger.logNumber(LoggingKey.PowerCellGenevaPower, desiredGenevaMotorPower);
        this.logger.logBoolean(LoggingKey.PowerCellIsIntaking, isIntaking);
        this.logger.logBoolean(LoggingKey.PowerCellIntakeExtended, this.intakeExtended);
    }

    @Override
    public void stop()
    {
        //this.genevaMotor.stop();
        this.rollerMotor.stop();
        //this.rollerMotorOuter.stop();
        //this.innerHood.set(DoubleSolenoidValue.Off);
        //this.outerHood.set(DoubleSolenoidValue.Off);
        //this.kickerSolenoid.set(DoubleSolenoidValue.Off);
        //this.intakeSolenoid.set(DoubleSolenoidValue.Off);
        //this.turret.stop();
        //this.flyWheel.stop();
    }

    @Override
    public void setDriver(Driver driver)
    {
        this.driver = driver;
    }

    // public double getTurretPosition()
    // {
    //     return (this.turretPosition * HardwareConstants.POWERCELL_TURRET_TICKS_TO_DEGREES) - this.startingTurretOffsetAngle;
    // }

    // public double getFlywheelVelocity()
    // {
    //     return this.flywheelVelocity;
    // }

    // public double getFlywheelVelocitySetpoint()
    // {
    //     return this.flywheelVelocitySetpoint;
    // }

    // public int getCurrentCarouselIndex()
    // {
    //     return this.currentCarouselIndex;
    // }

    // public boolean hasPowerCell(int index)
    // {
    //     return this.hasPowerCell[index];
    // }

    // public boolean hasAnyPowerCell()
    // {
    //     for (boolean slotHasPowerCell : this.hasPowerCell)
    //     {
    //         if (slotHasPowerCell)
    //         {
    //             return true;
    //         }
    //     }

    //     return false;
    // }
    
    // public boolean isFlywheelSpunUp()
    // {
    //     return this.flywheelVelocitySetpoint > 0.0 && Math.abs(this.flywheelError) <= TuningConstants.POWERCELL_FLYWHEEL_ALLOWABLE_ERROR_RANGE;
    // }

    // private double getClosestAngleInRange(double desiredAngle, double currentAngle, double minRangeValue, double maxRangeValue)
    // {
    //     double multiplicand = Math.floor(currentAngle / 360.0);

    //     double[] closeRotations =
    //     {
    //         (desiredAngle + 360.0 * (multiplicand - 1.0)),
    //         (desiredAngle + 360.0 * multiplicand),
    //         (desiredAngle + 360.0 * (multiplicand + 1.0)),
    //     };

    //     double best = currentAngle;
    //     double bestDistance = Double.POSITIVE_INFINITY;
    //     for (int i = 0; i < 3; i++)
    //     {
    //         double angle = closeRotations[i];
    //         if (Helpers.WithinRange(angle, minRangeValue, maxRangeValue))
    //         {
    //             double angleDistance = Math.abs(currentAngle - angle);
    //             if (angleDistance < bestDistance)
    //             {
    //                 best = angle;
    //                 bestDistance = angleDistance;
    //             }
    //         }
    //     }

    //     return best;
    // }

    // private enum CarouselState
    // {
    //     Indexing,
    //     Stationary,
    //     MovingToNext,
    //     MovingToPrevious,
    // }
}