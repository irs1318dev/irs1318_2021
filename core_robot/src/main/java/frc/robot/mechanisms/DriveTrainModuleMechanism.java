/*
 * DriveTrainModuleMechanism
 * 
 * authors: Will, Vanshika, Arushi
 * 
 * Started idk sometime in september
*/

package frc.robot.mechanisms;

import javax.inject.Singleton;

import frc.robot.*;
import frc.robot.common.*;
import frc.robot.common.robotprovider.*;
import frc.robot.driver.*;
import frc.robot.driver.common.Driver;

import com.google.inject.Inject;

@Singleton
public class DriveTrainModuleMechanism implements IMechanism
{
    private static final int pidSlotId = 0;
    private static final int FRAME_PERIOD_MS = 5;

    private static final double POWERLEVEL_MIN = -1.0;
    private static final double POWERLEVEL_MAX = 1.0;

    private final ILogger logger;
    private final IAnalogInput absoluteEncoder;

    private final ITalonFX driveMotor;
    private final ITalonFX angleMotor;

    private Driver driver;

    private double driveVelocity;
    private double driveError;
    private int drivePosition;
    private double angleVelocity;
    private double angleError;
    private int anglePosition;

    private double encoderVoltage;
    private double encoderAngle;

    @Inject
    public DriveTrainModuleMechanism(
        LoggingManager logger,
        IRobotProvider provider)
    {
        this.logger = logger;
        
        this.angleMotor = provider.getTalonFX(ElectronicsConstants.DRIVETRAIN_ANGLE_MOTOR_1_CAN_ID);
        this.angleMotor.setInvertOutput(HardwareConstants.DRIVETRAIN_ANGLE_MOTOR_1_INVERT_OUTPUT);
        this.angleMotor.setInvertSensor(HardwareConstants.DRIVETRAIN_ANGLE_MOTOR_1_INVERT_SENSOR);
        this.angleMotor.setNeutralMode(MotorNeutralMode.Brake);
        this.angleMotor.setSensorType(TalonXFeedbackDevice.IntegratedSensor);
        //this.angleMotor.setPosition((int)this.encoderAngle); //would this work?
        this.angleMotor.setPIDF(
            TuningConstants.DRIVETRAIN_ANGLE_MOTOR_1_POSITION_PID_KP,
            TuningConstants.DRIVETRAIN_ANGLE_MOTOR_1_POSITION_PID_KI,
            TuningConstants.DRIVETRAIN_ANGLE_MOTOR_1_POSITION_PID_KD,
            TuningConstants.DRIVETRAIN_ANGLE_MOTOR_1_POSITION_PID_KF,
            DriveTrainModuleMechanism.pidSlotId);

        this.driveMotor = provider.getTalonFX(ElectronicsConstants.DRIVETRAIN_DRIVE_MOTOR_1_CAN_ID);
        this.driveMotor.setNeutralMode(MotorNeutralMode.Brake);
        this.driveMotor.setInvertOutput(HardwareConstants.DRIVETRAIN_DRIVE_MOTOR_1_INVERT_OUTPUT);
        this.driveMotor.setInvertSensor(HardwareConstants.DRIVETRAIN_DRIVE_MOTOR_1_INVERT_SENSOR);
        this.driveMotor.setSensorType(TalonXFeedbackDevice.IntegratedSensor);
        this.driveMotor.setFeedbackFramePeriod(DriveTrainModuleMechanism.FRAME_PERIOD_MS);
        this.driveMotor.setPIDFFramePeriod(DriveTrainModuleMechanism.FRAME_PERIOD_MS);
        this.driveMotor.configureVelocityMeasurements(10, 32);
        this.driveMotor.setPIDF(
            TuningConstants.DRIVETRAIN_DRIVE_MOTOR_1_VELOCITY_PID_KP,
            TuningConstants.DRIVETRAIN_DRIVE_MOTOR_1_VELOCITY_PID_KI,
            TuningConstants.DRIVETRAIN_DRIVE_MOTOR_1_VELOCITY_PID_KD,
            TuningConstants.DRIVETRAIN_DRIVE_MOTOR_1_VELOCITY_PID_KF,
            DriveTrainModuleMechanism.pidSlotId);
        this.driveMotor.setVoltageCompensation(
            TuningConstants.DRIVETRAIN_VOLTAGE_COMPENSATION_ENABLED,
            TuningConstants.DRIVETRAIN_VOLTAGE_COMPENSATION);
        this.driveMotor.setSupplyCurrentLimit(
            TuningConstants.DRIVETRAIN_SUPPLY_CURRENT_LIMITING_ENABLED,
            TuningConstants.DRIVETRAIN_SUPPLY_CURRENT_MAX,
            TuningConstants.DRIVETRAIN_SUPPLY_TRIGGER_CURRENT,
            TuningConstants.DRIVETRAIN_SUPPLY_TRIGGER_DURATION);

        if (TuningConstants.DRIVETRAIN_USE_PID)
        {
            this.angleMotor.setControlMode(TalonSRXControlMode.Position);
            this.driveMotor.setControlMode(TalonSRXControlMode.Velocity);
        }
        else
        {
            this.angleMotor.setControlMode(TalonSRXControlMode.PercentOutput);
            this.driveMotor.setControlMode(TalonSRXControlMode.PercentOutput);
        }

        this.absoluteEncoder = provider.getAnalogInput(ElectronicsConstants.DRIVETRAIN_ABSOLUTE_ENCODER_1_ANALOG_INPUT);
    }

    /**
     * get the velocity from the drive encoder
     * @return a value indicating the velocity
     */
    public double getDriveVelocity()
    {
        return this.driveVelocity;
    }

    /**
     * get the velocity from the angle encoder
     * @return a value indicating the velocity
     */
    public double getAngleVelocity()
    {
        return this.angleVelocity;
    }

    /**
     * get the distance from the drive encoder
     * @return a value indicating the distance
     */
    public double getDriveError()
    {
        return this.driveError;
    }

    /**
     * get the distance from the angle encoder
     * @return a value indicating the distance
     */
    public double getAngleError()
    {
        return this.angleError;
    }

    /**
     * get the ticks from the drive encoder
     * @return a value indicating the number of ticks we are at
     */
    public int getDrivePosition()
    {
        return this.drivePosition;
    }

    /**
     * get the ticks from the angle encoder
     * @return a value indicating the number of ticks we are at
     */
    public int getAnglePosition()
    {
        return this.anglePosition;
    }

    @Override
    public void setDriver(Driver driver)
    {
        this.driver = driver;
    }

    @Override
    public void readSensors()
    {
        this.encoderVoltage = this.absoluteEncoder.getVoltage();
        this.encoderAngle = this.encoderVoltage * HardwareConstants.DRIVETRAIN_ENCODER_DEGREES_PER_VOLT;
        
        this.driveVelocity = this.driveMotor.getVelocity();
        this.angleVelocity = this.angleMotor.getVelocity();

        this.drivePosition = this.driveMotor.getPosition();
        this.anglePosition = this.angleMotor.getPosition();

        this.driveError = this.driveMotor.getError();
        this.angleError = this.angleMotor.getError();

        this.logger.logNumber(LoggingKey.DriveTrainDriveVelocity1, this.driveVelocity);
        this.logger.logNumber(LoggingKey.DriveTrainDriveError1, this.driveError);
        this.logger.logNumber(LoggingKey.DriveTrainDrivePosition1, this.drivePosition);
        this.logger.logNumber(LoggingKey.DriveTrainAngleVelocity1, this.angleVelocity);
        this.logger.logNumber(LoggingKey.DriveTrainAngleError1, this.angleError);
        this.logger.logNumber(LoggingKey.DriveTrainAnglePosition1, this.anglePosition);
        this.logger.logNumber(LoggingKey.DriveTrainAbsoluteEncoderPosition1, this.encoderAngle);
    }

    public void update()
    {
        Setpoint setpoint;
        if (TuningConstants.DRIVETRAIN_USE_PID)
        {
            setpoint = this.calculateSetpoint();
        }
        else
        {
            setpoint = this.calculateNonPIDSetpoint();   
        }

        double driveSetpoint = setpoint.getDrive();
        double angleSetpoint = setpoint.getAngle();

        this.logger.logNumber(LoggingKey.DriveTrainDriveVelocityGoal1, driveSetpoint);
        this.logger.logNumber(LoggingKey.DriveTrainAnglePositionGoal1, angleSetpoint);

        // apply the setpoints to the motors
        this.driveMotor.set(driveSetpoint);
        this.angleMotor.set(angleSetpoint);

        if (this.driver.getDigital(DigitalOperation.DriveTrainReset))
        {
            this.angleMotor.reset();
            this.driveMotor.reset();
        }
    }

    public void stop()
    {
        this.driveMotor.stop();
        this.angleMotor.stop();

        this.driveVelocity = 0.0;
        this.driveError = 0.0;
        this.drivePosition = 0;
        this.angleVelocity = 0.0;
        this.angleError = 0.0;
        this.anglePosition = 0;
    }

    private Setpoint calculateSetpoint()
    {
        double driveVelocityGoal = 0.0;
        double anglePositionGoal = 0.0;

        double turnX = this.driver.getAnalog(AnalogOperation.DriveTrainTurnX);
        double turnY = this.driver.getAnalog(AnalogOperation.DriveTrainTurnY);
        double forwardVelocity = this.driver.getAnalog(AnalogOperation.DriveTrainMoveForward);

        driveVelocityGoal = forwardVelocity;
        anglePositionGoal = (Math.atan2(turnX, turnY) * Helpers.RADIANS_TO_DEGREES);

        driveVelocityGoal = this.applyPowerLevelRange(driveVelocityGoal);

        Helpers.EnforceRange(anglePositionGoal, -180.0, 180.0);
        this.assertPowerLevelRange(driveVelocityGoal, "drive");

        driveVelocityGoal *= TuningConstants.DRIVETRAIN_DRIVE_MOTOR_1_VELOCITY_PID_KS;
        anglePositionGoal *= TuningConstants.DRIVETRAIN_ANGLE_MOTOR_1_POSITION_PID_KS;

        // if we are using PID, then we base the setpoint on the max velocity
        return new Setpoint(driveVelocityGoal, anglePositionGoal);
    }

    private Setpoint calculateNonPIDSetpoint()
    {
        double driveOutput = 0.0;
        double angleOutput = 0.0;

        double turnX = this.driver.getAnalog(AnalogOperation.DriveTrainTurnX);
        //ouble turnY = this.driver.getAnalog(AnalogOperation.DriveTrainTurnY);
        double forwardVelocity = this.driver.getAnalog(AnalogOperation.DriveTrainMoveForward);

        driveOutput = forwardVelocity;
        angleOutput = turnX;

        driveOutput = this.applyPowerLevelRange(driveOutput);
        angleOutput = this.applyPowerLevelRange(angleOutput);

        this.assertPowerLevelRange(angleOutput , "angle");
        this.assertPowerLevelRange(driveOutput, "drive");

        return new Setpoint(driveOutput, angleOutput);
    }

    private void assertPowerLevelRange(double powerLevel, String side)
    {
        if (powerLevel < DriveTrainModuleMechanism.POWERLEVEL_MIN)
        {
            if (TuningConstants.THROW_EXCEPTIONS)
            {
                throw new RuntimeException(side + " power level too low!");
            }

            return;
        }

        if (powerLevel > DriveTrainModuleMechanism.POWERLEVEL_MAX)
        {
            if (TuningConstants.THROW_EXCEPTIONS)
            {
                throw new RuntimeException(side + " power level too high!");
            }

            return;
        }
    }

    private double applyPowerLevelRange(double powerLevel)
    {
        return Helpers.EnforceRange(powerLevel, DriveTrainModuleMechanism.POWERLEVEL_MIN, DriveTrainModuleMechanism.POWERLEVEL_MAX);
    }

    private class Setpoint
    {
        private double angle;
        private double drive;

        /**
         * Initializes a new Setpoint
         * @param drive value to apply
         * @param angle value to apply
         */
        public Setpoint(double drive, double angle)
        {
            this.drive = drive;
            this.angle = angle;
        }

        /**
         * gets the drive setpoint
         * @return drive setpoint value
         */
        public double getDrive()
        {
            return this.drive;
        }

        /**
         * gets the angle setpoint
         * @return angle setpoint value
         */
        public double getAngle()
        {
            return this.angle;
        }
    }
}
