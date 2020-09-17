package frc.robot.mechanisms;

import java.lang.invoke.InjectedProfile;

import javax.inject.Singleton;

import frc.robot.*;
import frc.robot.common.*;
import frc.robot.common.robotprovider.*;
import frc.robot.driver.*;
import frc.robot.driver.common.Driver;

import com.google.inject.Inject;

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
        this.angleMotor.setPosition(0);
        this.angleMotor.setControlMode(TalonSRXControlMode.Position);
        this.angleMotor.setPIDF(
            TuningConstants.ANGLE_MOTOR_POSITION_PID_KP,
            TuningConstants.ANGLE_MOTOR_POSITION_PID_KI,
            TuningConstants.ANGLE_MOTOR_POSITION_PID_KD,
            TuningConstants.ANGLE_MOTOR_POSITION_PID_KF,
            DriveTrainMechanism.slotId);

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
        this.setControlMode();
    }

    @Override
    public void readSensors()
    {
        double encoderVoltage = this.absoluteEncoder.getVoltage();
        double encoderAngle = encoderVoltage * 72;
        
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

        this.setControlMode();
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

    public void setControlMode()
    {
        //Can we define this in the beginning 
    }

    private Setpoint calculateSetpoint()
    {
        double driveVelocityGoal = 0.0;
        double anglePositionGoal = 0.0;

        double turnAngle = this.driver.getAnalog(AnalogOperation.DriveTrainTurn);
        double forwardVelocity = this.driver.getAnalog(AnalogOperation.DriveTrainMoveForward);

        driveVelocityGoal = forwardVelocity * TuningConstants.DRIVETRAIN_K1;
        anglePositionGoal = turnAngle * TuningConstants.DRIVETRAIN_K2;

        driveVelocityGoal = driveVelocityGoal * TuningConstants.DRIVETRAIN_MAX_POWER_LEVEL;
        anglePositionGoal = anglePositionGoal * TuningConstants.DRIVETRAIN_MAX_POWER_LEVEL;

        double drive = this.applyPowerLevelRange(driveVelocityGoal);
        double angle = this.applyPowerLevelRange(angleVelocityGoal);

        this.assertPowerLevelRange(drive, "drive");
        this.assertPowerLevelRange(angle, "angle");

        // if we are using PID, then we base the setpoint on the max velocity
        if (this.usePID)
        {
            drive *= TuningConstants.ANGLE_MOTOR_POSITION_PID_KS;
            angle *= TuningConstants.DRIVE_MOTOR_VELOCITY_PID_KS;
        }

        return new Setpoint(drive, angle);
    }


}
