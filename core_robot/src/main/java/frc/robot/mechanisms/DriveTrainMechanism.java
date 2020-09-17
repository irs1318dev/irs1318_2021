package frc.robot.mechanisms;

import java.lang.invoke.InjectedProfile;

import javax.inject.Singleton;

import frc.robot.*;
import frc.robot.common.*;
import frc.robot.common.robotprovider.*;
import frc.robot.driver.*;
import frc.robot.driver.common.Driver;

import com.google.inject.Inject;

/*
DriveTrainMechanism

authors: Vanshika, Arushi

Started idk sometime in september
*/


@Singleton
public class DriveTrainMechanism implements IMechanism {

    private static final int pidSlotId = 0;
    private static final int FRAME_PERIOD_MS = 5;

    private static final double POWERLEVEL_MIN = -1.0;
    private static final double POWERLEVEL_MAX = 1.0;

    private final ILogger logger;
    private final ITimer timer;
    private final IAnalogInput absoluteEncoder;

    private final ITalonFX driveMotor;
    private final ITalonFX angleMotor;

    private Driver driver;

    private PIDHandler drivePID;
    private PIDHandler anglePID;

    private boolean usePID;
    private boolean useBrakeMode;

    private double driveVelocity;
    private double driveError;
    private int drivePosition;
    private double angleVelocity;
    private double angleError;
    private int anglePosition;

    private double encoderVoltage;
    private double encoderAngle;


    @Inject
    public DriveTrainMechanism(
    LoggingManager logger,
    IRobotProvider provider,
    ITimer timer) {

        this.logger = logger;
        this.timer = timer;
        
        this.angleMotor = provider.getTalonSRX(ElectronicsConstants.ANGLE_MOTOR_CAN_ID);
        this.angleMotor.setInvertOutput(HardwareConstants.ANGLE_MOTOR_INVERT_OUTPUT);
        this.angleMotor.setInvertSensor(HardwareConstants.ANGLE_MOTOR_INVERT_SENSOR);
        this.angleMotor.setNeutralMode(MotorNeutralMode.Brake);
        //this.angleMotor.setSensorType();
        this.angleMotor.setPosition((int)this.encoderAngle); //would this work?
        this.angleMotor.setControlMode(TalonSRXControlMode.Position);
        this.angleMotor.setPIDF(
            TuningConstants.ANGLE_MOTOR_POSITION_PID_KP,
            TuningConstants.ANGLE_MOTOR_POSITION_PID_KI,
            TuningConstants.ANGLE_MOTOR_POSITION_PID_KD,
            TuningConstants.ANGLE_MOTOR_POSITION_PID_KF,
            DriveTrainMechanism.pidSlotId);

        this.driveMotor = provider.getTalonFX(ElectronicsConstants.DRIVE_MOTOR_CAN_ID);
        this.driveMotor.setNeutralMode(MotorNeutralMode.Brake);
        this.driveMotor.setInvertOutput(HardwareConstants.DRIVE_MOTOR_INVERT_OUTPUT);
        this.driveMotor.setInvertSensor(HardwareConstants.DRIVE_MOTOR_INVERT_SENSOR);
        this.driveMotor.setSensorType(TalonXFeedbackDevice.IntegratedSensor);
        this.driveMotor.setFeedbackFramePeriod(DriveTrainMechanism.FRAME_PERIOD_MS);
        this.driveMotor.setPIDFFramePeriod(DriveTrainMechanism.FRAME_PERIOD_MS);
        this.driveMotor.configureVelocityMeasurements(10, 32);
        this.driveMotor.setPIDF(
            TuningConstants.DRIVE_MOTOR_VELOCITY_PID_KP,
            TuningConstants.DRIVE_MOTOR_VELOCITY_PID_KI,
            TuningConstants.DRIVE_MOTOR_VELOCITY_PID_KD,
            TuningConstants.DRIVE_MOTOR_VELOCITY_PID_KF,
            DriveTrainMechanism.pidSlotId);
        this.driveMotor.setVoltageCompensation(
            TuningConstants.DRIVETRAIN_VOLTAGE_COMPENSATION_ENABLED,
            TuningConstants.DRIVETRAIN_VOLTAGE_COMPENSATION);
        this.driveMotor.setSupplyCurrentLimit(
            TuningConstants.DRIVETRAIN_SUPPLY_CURRENT_LIMITING_ENABLED,
            TuningConstants.DRIVETRAIN_SUPPLY_CURRENT_MAX,
            TuningConstants.DRIVETRAIN_SUPPLY_TRIGGER_CURRENT,
            TuningConstants.DRIVETRAIN_SUPPLY_TRIGGER_DURATION);

        this.absoluteEncoder = provider.getAnalogInput(ElectronicsConstants.ABSOLUTE_ENCODER);

    }
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
        this.encoderAngle = this.encoderVoltage * 72;
        
        this.driveVelocity = this.driveMotor.getVelocity();
        this.angleVelocity = this.angleMotor.getVelocity();

        this.drivePosition = this.driveMotor.getPosition();
        this.anglePosition = this.angleMotor.getPosition();

        this.driveError = this.driveMotor.getError();
        this.angleError = this.angleMotor.getError();

        this.logger.logNumber(LoggingKey.SwerveDriveVelocity, this.driveVelocity);
        this.logger.logNumber(LoggingKey.SwerveDriveError, this.driveError);
        this.logger.logNumber(LoggingKey.SwerveAnglePosition, this.drivePosition);
        this.logger.logNumber(LoggingKey.SwerveAngleVelocity, this.angleVelocity);
        this.logger.logNumber(LoggingKey.SwerveAngleError, this.angleError);
        this.logger.logNumber(LoggingKey.SwerveAnglePosition, this.anglePosition);
    }

    public void update()
    {
        Setpoint setpoint = this.calculateSetpoint();

        double driveSetpoint = setpoint.getDrive();
        double angleSetpoint = setpoint.getAngle();

        this.logger.logNumber(LoggingKey.SwerveDriveVelocityGoal, driveSetpoint);
        this.logger.logNumber(LoggingKey.SwerveAnglePositionGoal, angleSetpoint);

        // apply the setpoints to the motors
        this.driveMotor.set(driveSetpoint);
        this.angleMotor.set(angleSetpoint);
    }

    public void stop()
    {
        this.driveMotor.stop();
        this.angleMotor.stop();

        this.driveMotor.reset();
        this.angleMotor.reset();

        this.drivePID.reset();
        this.anglePID.reset();

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

        driveVelocityGoal = forwardVelocity * TuningConstants.DRIVETRAIN_K1;
        anglePositionGoal = (Math.atan2(turnX, turnY)/Math.PI) * 180;

        driveVelocityGoal = driveVelocityGoal * TuningConstants.DRIVETRAIN_MAX_POWER_LEVEL;

        driveVelocityGoal = this.applyPowerLevelRange(driveVelocityGoal);

        this.assertPowerLevelRange(drive, "drive");

        // if we are using PID, then we base the setpoint on the max velocity
        if (this.usePID)
        {
            drive *= TuningConstants.DRIVE_MOTOR_VELOCITY_PID_KS;
        }

        return new Setpoint(driveVelocityGoal, anglePositionGoal);
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
