/**
 * DriveTrainMechanism
 * 
 * authors: Will, Vanshika, Arushi
 * 
 * Started idk sometime in september
 * 
 * dO yOu ReMeMbEr
 * tHe 21sT nIgHt oF sEpTeMbEr
**/

package frc.robot.mechanisms;

import frc.robot.*;
import frc.robot.common.*;
import frc.robot.common.robotprovider.*;
import frc.robot.driver.*;
import frc.robot.driver.common.Driver;

import com.google.inject.Inject;
import com.google.inject.Singleton;

@Singleton
public class DriveTrainMechanism implements IMechanism
{
    private static final int NUM_MODULES = 4;

    private static final int pidSlotId = 0;
    private static final int FRAME_PERIOD_MS = 5;

    private static final double POWERLEVEL_MIN = -1.0;
    private static final double POWERLEVEL_MAX = 1.0;

    private final LoggingKey[] ENCODER_ANGLE_LOGGING_KEYS = { LoggingKey.DriveTrainAbsoluteEncoderAngle1, LoggingKey.DriveTrainAbsoluteEncoderAngle2, LoggingKey.DriveTrainAbsoluteEncoderAngle3, LoggingKey.DriveTrainAbsoluteEncoderAngle4 };
    private final LoggingKey[] DRIVE_VELOCITY_LOGGING_KEYS = { LoggingKey.DriveTrainDriveVelocity1, LoggingKey.DriveTrainDriveVelocity2, LoggingKey.DriveTrainDriveVelocity3, LoggingKey.DriveTrainDriveVelocity4 };
    private final LoggingKey[] DRIVE_POSITION_LOGGING_KEYS = { LoggingKey.DriveTrainDrivePosition1, LoggingKey.DriveTrainDrivePosition2, LoggingKey.DriveTrainDrivePosition3, LoggingKey.DriveTrainDrivePosition4 };
    private final LoggingKey[] DRIVE_ERROR_LOGGING_KEYS = { LoggingKey.DriveTrainDriveError1, LoggingKey.DriveTrainDriveError2, LoggingKey.DriveTrainDriveError3, LoggingKey.DriveTrainDriveError4 };
    private final LoggingKey[] STEER_VELOCITY_LOGGING_KEYS = { LoggingKey.DriveTrainSteerVelocity1, LoggingKey.DriveTrainSteerVelocity2, LoggingKey.DriveTrainSteerVelocity3, LoggingKey.DriveTrainSteerVelocity4 };
    private final LoggingKey[] STEER_POSITION_LOGGING_KEYS = { LoggingKey.DriveTrainSteerPosition1, LoggingKey.DriveTrainSteerPosition2, LoggingKey.DriveTrainSteerPosition3, LoggingKey.DriveTrainSteerPosition4 };
    private final LoggingKey[] STEER_ANGLE_LOGGING_KEYS = { LoggingKey.DriveTrainSteerAngle1, LoggingKey.DriveTrainSteerAngle2, LoggingKey.DriveTrainSteerAngle3, LoggingKey.DriveTrainSteerAngle4 };
    private final LoggingKey[] STEER_ERROR_LOGGING_KEYS = { LoggingKey.DriveTrainSteerError1, LoggingKey.DriveTrainSteerError2, LoggingKey.DriveTrainSteerError3, LoggingKey.DriveTrainSteerError4 };
    private final LoggingKey[] DRIVE_GOAL_LOGGING_KEYS = { LoggingKey.DriveTrainDriveVelocityGoal1, LoggingKey.DriveTrainDriveVelocityGoal2, LoggingKey.DriveTrainDriveVelocityGoal3, LoggingKey.DriveTrainDriveVelocityGoal4 };
    private final LoggingKey[] STEER_GOAL_LOGGING_KEYS = { LoggingKey.DriveTrainSteerPositionGoal1, LoggingKey.DriveTrainSteerPositionGoal2, LoggingKey.DriveTrainSteerPositionGoal3, LoggingKey.DriveTrainSteerPositionGoal4 };

    private final double[] MODULE_OFFSET_X; // the x offsets of the swerve modules from the default center of rotation
    private final double[] MODULE_OFFSET_Y; // the y offsets of the swerve modules from the default center of rotation

    private final NavxManager navxManager;
    private final ILogger logger;
    private final ITimer timer;

    private final ITalonFX[] steerMotors;
    private final ITalonFX[] driveMotors;
    private final IAnalogInput[] absoluteEncoders;

    private Driver driver;

    private boolean fieldOriented;
    private PIDHandler omegaPID;
    private double desiredYaw;
    private boolean[] isDirectionSwapped;

    private PIDHandler pathOmegaPID;
    private PIDHandler pathXOffsetPID;
    private PIDHandler pathYOffsetPID;

    private double time;
    private double angle;
    private double xPosition;
    private double yPosition;
    private double deltaT;

    private double robotNavxYaw;
    private double[] driveVelocities;
    private double[] drivePositions;
    private double[] driveErrors;
    private double[] steerVelocities;
    private double[] steerPositions;
    private double[] steerAngles;
    private double[] steerErrors;
    private double[] encoderVoltages;
    private double[] encoderAngles;

    private Setpoint[] result;

    @Inject
    public DriveTrainMechanism(
        LoggingManager logger,
        IRobotProvider provider,
        NavxManager navxManager,
        ITimer timer)
    {
        this.timer = timer;
        this.logger = logger;
        this.navxManager = navxManager;

        this.steerMotors = new ITalonFX[DriveTrainMechanism.NUM_MODULES];
        this.driveMotors = new ITalonFX[DriveTrainMechanism.NUM_MODULES];
        this.absoluteEncoders = new IAnalogInput[DriveTrainMechanism.NUM_MODULES];
        for (int i = 0; i < DriveTrainMechanism.NUM_MODULES; i++)
        {
            this.driveMotors[i] = provider.getTalonFX(ElectronicsConstants.DRIVETRAIN_DRIVE_MOTOR_CAN_ID[i]);
            this.driveMotors[i].setNeutralMode(MotorNeutralMode.Brake); //
            this.driveMotors[i].setSensorType(TalonXFeedbackDevice.IntegratedSensor); //
            this.driveMotors[i].setFeedbackFramePeriod(DriveTrainMechanism.FRAME_PERIOD_MS); //
            this.driveMotors[i].setPIDFFramePeriod(DriveTrainMechanism.FRAME_PERIOD_MS); //
            this.driveMotors[i].setInvertOutput(HardwareConstants.DRIVETRAIN_DRIVE_MOTOR_INVERT_OUTPUT[i]);
            this.driveMotors[i].setInvertSensor(HardwareConstants.DRIVETRAIN_DRIVE_MOTOR_INVERT_SENSOR[i]);
            this.driveMotors[i].configureVelocityMeasurements(10, 32); //
            this.driveMotors[i].setPIDF(
                TuningConstants.DRIVETRAIN_DRIVE_MOTOR_VELOCITY_PID_KP[i],
                TuningConstants.DRIVETRAIN_DRIVE_MOTOR_VELOCITY_PID_KI[i],
                TuningConstants.DRIVETRAIN_DRIVE_MOTOR_VELOCITY_PID_KD[i],
                TuningConstants.DRIVETRAIN_DRIVE_MOTOR_VELOCITY_PID_KF[i],
                DriveTrainMechanism.pidSlotId);
            this.driveMotors[i].setVoltageCompensation(
                TuningConstants.DRIVETRAIN_VOLTAGE_COMPENSATION_ENABLED,
                TuningConstants.DRIVETRAIN_VOLTAGE_COMPENSATION);
            this.driveMotors[i].setSupplyCurrentLimit(
                TuningConstants.DRIVETRAIN_SUPPLY_CURRENT_LIMITING_ENABLED,
                TuningConstants.DRIVETRAIN_SUPPLY_CURRENT_MAX,
                TuningConstants.DRIVETRAIN_SUPPLY_TRIGGER_CURRENT,
                TuningConstants.DRIVETRAIN_SUPPLY_TRIGGER_DURATION);
            this.driveMotors[i].setControlMode(TalonSRXControlMode.Velocity);

            this.steerMotors[i] = provider.getTalonFX(ElectronicsConstants.DRIVETRAIN_STEER_MOTOR_CAN_ID[i]);
            this.steerMotors[i].setInvertOutput(HardwareConstants.DRIVETRAIN_STEER_MOTOR_INVERT_OUTPUT[i]);
            this.steerMotors[i].setInvertSensor(HardwareConstants.DRIVETRAIN_STEER_MOTOR_INVERT_SENSOR[i]);
            this.steerMotors[i].setNeutralMode(MotorNeutralMode.Brake);
            this.steerMotors[i].setSensorType(TalonXFeedbackDevice.IntegratedSensor);
            this.steerMotors[i].setPIDF(
                TuningConstants.DRIVETRAIN_STEER_MOTOR_POSITION_PID_KP[i],
                TuningConstants.DRIVETRAIN_STEER_MOTOR_POSITION_PID_KI[i],
                TuningConstants.DRIVETRAIN_STEER_MOTOR_POSITION_PID_KD[i],
                TuningConstants.DRIVETRAIN_STEER_MOTOR_POSITION_PID_KF[i],
                DriveTrainMechanism.pidSlotId);

            this.steerMotors[i].setControlMode(TalonSRXControlMode.Position);

            this.absoluteEncoders[i] = provider.getAnalogInput(ElectronicsConstants.DRIVETRAIN_ABSOLUTE_ENCODER_ANALOG_INPUT[i]);
        }

        this.driveVelocities = new double[DriveTrainMechanism.NUM_MODULES];
        this.drivePositions = new double[DriveTrainMechanism.NUM_MODULES];
        this.driveErrors = new double[DriveTrainMechanism.NUM_MODULES];
        this.steerVelocities = new double[DriveTrainMechanism.NUM_MODULES];
        this.steerPositions = new double[DriveTrainMechanism.NUM_MODULES];
        this.steerAngles = new double[DriveTrainMechanism.NUM_MODULES];
        this.steerErrors = new double[DriveTrainMechanism.NUM_MODULES];
        this.encoderVoltages = new double[DriveTrainMechanism.NUM_MODULES];
        this.encoderAngles = new double[DriveTrainMechanism.NUM_MODULES];

        this.isDirectionSwapped = new boolean[DriveTrainMechanism.NUM_MODULES];

        this.omegaPID = new PIDHandler(
            TuningConstants.DRIVETRAIN_OMEGA_POSITION_PID_KP,
            TuningConstants.DRIVETRAIN_OMEGA_POSITION_PID_KI,
            TuningConstants.DRIVETRAIN_OMEGA_POSITION_PID_KD,
            TuningConstants.DRIVETRAIN_OMEGA_POSITION_PID_KF,
            TuningConstants.DRIVETRAIN_OMEGA_POSITION_PID_KS,
            TuningConstants.DRIVETRAIN_OMEGA_MIN_OUTPUT,
            TuningConstants.DRIVETRAIN_OMEGA_MAX_OUTPUT,
            this.timer);

        this.pathOmegaPID = new PIDHandler(
            TuningConstants.DRIVETRAIN_PATH_OMEGA_POSITION_PID_KP,
            TuningConstants.DRIVETRAIN_PATH_OMEGA_POSITION_PID_KI,
            TuningConstants.DRIVETRAIN_PATH_OMEGA_POSITION_PID_KD,
            TuningConstants.DRIVETRAIN_PATH_OMEGA_POSITION_PID_KF,
            TuningConstants.DRIVETRAIN_PATH_OMEGA_POSITION_PID_KS,
            TuningConstants.DRIVETRAIN_PATH_OMEGA_MIN_OUTPUT,
            TuningConstants.DRIVETRAIN_PATH_OMEGA_MAX_OUTPUT,
            this.timer);

        this.pathXOffsetPID = new PIDHandler(
            TuningConstants.DRIVETRAIN_PATH_X_POSITION_PID_KP,
            TuningConstants.DRIVETRAIN_PATH_X_POSITION_PID_KI,
            TuningConstants.DRIVETRAIN_PATH_X_POSITION_PID_KD,
            TuningConstants.DRIVETRAIN_PATH_X_POSITION_PID_KF,
            TuningConstants.DRIVETRAIN_PATH_X_POSITION_PID_KS,
            TuningConstants.DRIVETRAIN_PATH_X_MIN_OUTPUT,
            TuningConstants.DRIVETRAIN_PATH_X_MAX_OUTPUT,
            this.timer);

        this.pathYOffsetPID = new PIDHandler(
            TuningConstants.DRIVETRAIN_PATH_Y_POSITION_PID_KP,
            TuningConstants.DRIVETRAIN_PATH_Y_POSITION_PID_KI,
            TuningConstants.DRIVETRAIN_PATH_Y_POSITION_PID_KD,
            TuningConstants.DRIVETRAIN_PATH_Y_POSITION_PID_KF,
            TuningConstants.DRIVETRAIN_PATH_Y_POSITION_PID_KS,
            TuningConstants.DRIVETRAIN_PATH_Y_MIN_OUTPUT,
            TuningConstants.DRIVETRAIN_PATH_Y_MAX_OUTPUT,
            this.timer);

        double a1 = -HardwareConstants.DRIVETRAIN_HORIZONTAL_WHEEL_CENTER_DISTANCE;
        double a2 = HardwareConstants.DRIVETRAIN_HORIZONTAL_WHEEL_CENTER_DISTANCE;
        double b1 = -HardwareConstants.DRIVETRAIN_VERTICAL_WHEEL_CENTER_DISTANCE;
        double b2 = HardwareConstants.DRIVETRAIN_VERTICAL_WHEEL_CENTER_DISTANCE;

        this.MODULE_OFFSET_X = new double[] { a1, a2, a2, a1 };
        this.MODULE_OFFSET_Y = new double[] { b1, b1, b2, b2 };
        this.result = new Setpoint[DriveTrainMechanism.NUM_MODULES];

        this.time = 0.0;
        this.angle = 0.0;
        this.xPosition = 0.0;
        this.yPosition = 0.0;
    }

    @Override
    public void setDriver(Driver driver)
    {
        this.driver = driver;
        this.time = timer.get();
    }

    @Override
    public void readSensors()
    {
        for (int i = 0; i < DriveTrainMechanism.NUM_MODULES; i++)
        {
            this.driveVelocities[i] = this.driveMotors[i].getVelocity();
            this.drivePositions[i] = this.driveMotors[i].getPosition();
            this.driveErrors[i] = this.driveMotors[i].getError();
            this.steerVelocities[i] = this.steerMotors[i].getVelocity();
            this.steerPositions[i] = this.steerMotors[i].getPosition();
            this.steerAngles[i] = Helpers.updateAngleRange(this.steerPositions[i] * HardwareConstants.DRIVETRAIN_STEER_PULSE_DISTANCE);
            this.steerErrors[i] = this.steerMotors[i].getError();
            this.encoderVoltages[i] = this.absoluteEncoders[i].getVoltage();
            this.encoderAngles[i] = Helpers.updateAngleRange(this.encoderVoltages[i] * HardwareConstants.DRIVETRAIN_ENCODER_DEGREES_PER_VOLT);

            this.logger.logNumber(this.DRIVE_VELOCITY_LOGGING_KEYS[i], this.driveVelocities[i]);
            this.logger.logNumber(this.DRIVE_POSITION_LOGGING_KEYS[i], this.drivePositions[i]);
            this.logger.logNumber(this.DRIVE_ERROR_LOGGING_KEYS[i], this.driveErrors[i]);
            this.logger.logNumber(this.STEER_VELOCITY_LOGGING_KEYS[i], this.steerVelocities[i]);
            this.logger.logNumber(this.STEER_POSITION_LOGGING_KEYS[i], this.steerPositions[i]);
            this.logger.logNumber(this.STEER_ANGLE_LOGGING_KEYS[i], this.steerAngles[i]);
            this.logger.logNumber(this.STEER_ERROR_LOGGING_KEYS[i], this.steerErrors[i]);
            this.logger.logNumber(this.ENCODER_ANGLE_LOGGING_KEYS[i], this.encoderAngles[i]);
        }

        double prevNavxYaw = this.robotNavxYaw;
        double prevTime = this.time;
        this.robotNavxYaw = this.navxManager.getAngle();
        this.time = this.timer.get();

        this.deltaT = this.time - prevTime;
        if (TuningConstants.DRIVETRAIN_USE_ODOMETRY)
        {
            double deltaNavxYaw = (this.robotNavxYaw - prevNavxYaw) / this.deltaT;
            this.calculateOdometry(deltaNavxYaw);
            this.logger.logNumber(LoggingKey.DriveTrainXPosition, this.xPosition);
            this.logger.logNumber(LoggingKey.DriveTrainYPosition, this.yPosition);
            this.logger.logNumber(LoggingKey.DriveTrainAngle, this.angle);
        }
    }

    public void update()
    {
        if (this.driver.getDigital(DigitalOperation.DriveTrainEnableFieldOrientation))
        {
            this.fieldOriented = true;
            this.desiredYaw = this.robotNavxYaw;
        }

        if (this.driver.getDigital(DigitalOperation.DriveTrainDisableFieldOrientation) ||
            !this.navxManager.getIsConnected())
        {
            this.fieldOriented = false;
        }

        if (this.driver.getDigital(DigitalOperation.PositionResetFieldOrientation))
        {
            this.robotNavxYaw = this.navxManager.getAngle();
            this.desiredYaw = this.robotNavxYaw;
        }

        if (this.driver.getDigital(DigitalOperation.DriveTrainReset))
        {
            for (int i = 0; i < DriveTrainMechanism.NUM_MODULES; i++)
            {
                this.driveMotors[i].setPosition(0);
                double angleDifference = (this.encoderAngles[i] - HardwareConstants.DRIVETRAIN_STEER_MOTOR_ABSOLUTE_OFFSET[i]);
                double tickDifference = angleDifference * HardwareConstants.DRIVETRAIN_STEER_TICKS_PER_DEGREE;
                this.steerMotors[i].setPosition((int)tickDifference);

                this.drivePositions[i] = 0;
                this.steerPositions[i] = (int)tickDifference;
                this.steerAngles[i] = angleDifference % 360.0;
            }
        }

        this.calculateSetpoints();
        for (int i = 0; i < DriveTrainMechanism.NUM_MODULES; i++)
        {
            Setpoint current = this.result[i];
            Double steerSetpoint = current.getAngle();
            double driveSetpoint = current.getDrive();

            this.logger.logNumber(this.DRIVE_GOAL_LOGGING_KEYS[i], driveSetpoint);
            this.driveMotors[i].set(driveSetpoint);

            if (steerSetpoint != null)
            {
                this.logger.logNumber(this.STEER_GOAL_LOGGING_KEYS[i], steerSetpoint);
                this.steerMotors[i].set(steerSetpoint);
            }
        }
    }

    public void stop()
    {
        this.omegaPID.reset();
        for (int i = 0; i < DriveTrainMechanism.NUM_MODULES; i++)
        {
            this.driveMotors[i].stop();
            this.steerMotors[i].stop();
        }
    }

    public Pose2d getPose()
    {
        return new Pose2d(this.xPosition, this.yPosition, this.robotNavxYaw);
    }

    private void calculateSetpoints()
    {
        // calculate center velocity and turn velocity based on our current control mode:
        double rotationCenterA;
        double rotationCenterB;
        double centerVelocityX;
        double centerVelocityY;
        double omega;
        if (this.driver.getDigital(DigitalOperation.DriveTrainPathMode))
        {
            // path mode doesn't support rotation centers besides the robot center
            rotationCenterA = 0.0;
            rotationCenterB = 0.0;

            double xGoal = this.driver.getAnalog(AnalogOperation.DriveTrainPathXGoal);
            double yGoal = this.driver.getAnalog(AnalogOperation.DriveTrainPathYGoal);
            double angleReference = this.driver.getAnalog(AnalogOperation.DriveTrainTurnAngleReference);
            double angleGoal = this.driver.getAnalog(AnalogOperation.DriveTrainTurnAngleGoal);
            double velocityGoal = this.driver.getAnalog(AnalogOperation.DriveTrainPathVelocityGoal);

            System.out.println("velocityGoal " + velocityGoal + " angleGoal " + angleGoal);

            // convert velocity goal from in/sec to percentage of max velocity
            velocityGoal *= TuningConstants.DRIVETRAIN_VELOCITY_TO_PERCENTAGE;

            // convert velocity goal to be field-relative (from being angleGoal relative)
            double fieldVelocityX = velocityGoal * Helpers.cosd(angleGoal);
            double fieldVelocityY = velocityGoal * Helpers.sind(angleGoal);

            System.out.println("fieldVelocityX " + fieldVelocityX + " fieldVelocityY " + fieldVelocityY);

            // add correction for x/y drift
            // fieldVelocityX += this.pathXOffsetPID.calculatePosition(xGoal, this.xPosition);
            // fieldVelocityY += this.pathYOffsetPID.calculatePosition(yGoal, this.yPosition);

            // convert velocity to be robot-oriented
            centerVelocityX = Helpers.cosd(this.robotNavxYaw) * fieldVelocityX + Helpers.sind(this.robotNavxYaw) * fieldVelocityX;
            centerVelocityY = Helpers.cosd(this.robotNavxYaw) * fieldVelocityY - Helpers.sind(this.robotNavxYaw) * fieldVelocityY;

            AnglePair anglePair = AnglePair.getClosestAngle(angleGoal + angleReference, this.robotNavxYaw, false);
            this.desiredYaw = anglePair.getAngle();

            this.logger.logNumber(LoggingKey.DriveTrainDesiredAngle, this.desiredYaw);
            omega = this.pathOmegaPID.calculatePosition(this.desiredYaw, this.robotNavxYaw);
        }
        else
        {
            // get the center of rotation modifying control values
            rotationCenterA = this.driver.getAnalog(AnalogOperation.DriveTrainRotationA);
            rotationCenterB = this.driver.getAnalog(AnalogOperation.DriveTrainRotationB);

            // get the center velocity control values (could be field-oriented or robot-oriented center velocity)
            double centerVelocityXRaw = this.driver.getAnalog(AnalogOperation.DriveTrainMoveSide);
            double centerVelocityYRaw = this.driver.getAnalog(AnalogOperation.DriveTrainMoveForward);

            if (this.fieldOriented)
            {
                centerVelocityX = Helpers.cosd(this.robotNavxYaw) * centerVelocityXRaw + Helpers.sind(this.robotNavxYaw) * centerVelocityYRaw;
                centerVelocityY = Helpers.cosd(this.robotNavxYaw) * centerVelocityYRaw - Helpers.sind(this.robotNavxYaw) * centerVelocityXRaw;

                double yawGoal = this.driver.getAnalog(AnalogOperation.DriveTrainTurnAngleGoal);
                if (yawGoal != TuningConstants.MAGIC_NULL_VALUE)
                {
                    AnglePair anglePair = AnglePair.getClosestAngle(yawGoal, this.robotNavxYaw, false);
                    this.desiredYaw = anglePair.getAngle();
                }
                else
                {
                    double turnSpeed = this.driver.getAnalog(AnalogOperation.DriveTrainTurnSpeed);
                    this.desiredYaw += turnSpeed * TuningConstants.DRIVETRAIN_TURN_GOAL_VELOCITY;
                }

                this.logger.logNumber(LoggingKey.DriveTrainDesiredAngle, this.desiredYaw);
                omega = this.omegaPID.calculatePosition(this.desiredYaw, this.robotNavxYaw);
            }
            else
            {
                centerVelocityX = centerVelocityXRaw;
                centerVelocityY = centerVelocityYRaw;
                omega = this.driver.getAnalog(AnalogOperation.DriveTrainTurnSpeed);
            }
        }

        omega *= TuningConstants.DRIVETRAIN_TURN_SCALE;
        for (int i = 0; i < DriveTrainMechanism.NUM_MODULES; i++)
        {
            double moduleVelocityX = centerVelocityX + omega * (this.MODULE_OFFSET_Y[i] + rotationCenterB);
            double moduleVelocityY = centerVelocityY - omega * (this.MODULE_OFFSET_X[i] + rotationCenterA);

            Double moduleSteerPositionGoal;
            double moduleDriveVelocityGoal;
            if (TuningConstants.DRIVETRAIN_SKIP_ANGLE_ON_ZERO_VELOCITY
                    && Helpers.WithinDelta(moduleVelocityX, 0.0, TuningConstants.DRIVETRAIN_SKIP_ANGLE_ON_ZERO_DELTA)
                    && Helpers.WithinDelta(moduleVelocityY, 0.0, TuningConstants.DRIVETRAIN_SKIP_ANGLE_ON_ZERO_DELTA))
            {
                moduleDriveVelocityGoal = 0.0;
                moduleSteerPositionGoal = null;
            }
            else
            {
                moduleDriveVelocityGoal = Math.sqrt(moduleVelocityX * moduleVelocityX + moduleVelocityY * moduleVelocityY);

                moduleSteerPositionGoal = Helpers.EnforceRange(Helpers.atan2d(-moduleVelocityX, moduleVelocityY), -180.0, 180.0);
                double currentAngle = this.steerPositions[i] * HardwareConstants.DRIVETRAIN_STEER_PULSE_DISTANCE;
                AnglePair anglePair = AnglePair.getClosestAngle(moduleSteerPositionGoal, currentAngle, true);
                moduleSteerPositionGoal = anglePair.getAngle() * TuningConstants.DRIVETRAIN_STEER_MOTOR_POSITION_PID_KS;
                this.isDirectionSwapped[i] = anglePair.getSwapDirection();
            }

            moduleDriveVelocityGoal = this.applyPowerLevelRange(moduleDriveVelocityGoal);

            this.assertPowerLevelRange(moduleDriveVelocityGoal, "drive");

            moduleDriveVelocityGoal *= TuningConstants.DRIVETRAIN_DRIVE_MOTOR_VELOCITY_PID_KS;
            if (this.isDirectionSwapped[i])
            {
                moduleDriveVelocityGoal *= -1.0;
            }

            this.result[i] = new Setpoint(moduleDriveVelocityGoal, moduleSteerPositionGoal);
        }
    }

    public void calculateOdometry(double deltaNavxYaw)
    {
        double navxOmega = deltaNavxYaw / this.deltaT; // in degrees

        double omega1; // in radians / second
        double horizontalCenterDistance1;
        double verticalCenterDistance1;
        if (Helpers.AnglePairWithinDelta(this.steerAngles[0], this.isDirectionSwapped[0] ^ (this.driveVelocities[0] >= 0.0), this.steerAngles[1], this.isDirectionSwapped[1] ^ (this.driveVelocities[1] >= 0.0), 0.01))
        {
            omega1 = 0.0;
            horizontalCenterDistance1 = Double.POSITIVE_INFINITY;
            verticalCenterDistance1 = 0.0;
        }
        else
        {
            if (Helpers.WithinDelta(Math.abs(this.steerAngles[0]), 90.0, 0.005))
            {
                horizontalCenterDistance1 = HardwareConstants.DRIVETRAIN_HORIZONTAL_WHEEL_CENTER_DISTANCE;
                verticalCenterDistance1 = HardwareConstants.DRIVETRAIN_HORIZONTAL_WHEEL_SEPERATION_DISTANCE * Helpers.tand(-this.steerAngles[1]) - HardwareConstants.DRIVETRAIN_VERTICAL_WHEEL_CENTER_DISTANCE;
            }
            else if (Helpers.WithinDelta(Math.abs(this.steerAngles[1]), 90.0, 0.005))
            {
                horizontalCenterDistance1 = -HardwareConstants.DRIVETRAIN_HORIZONTAL_WHEEL_CENTER_DISTANCE;
                verticalCenterDistance1 = HardwareConstants.DRIVETRAIN_HORIZONTAL_WHEEL_SEPERATION_DISTANCE * Helpers.tand(this.steerAngles[0]) - HardwareConstants.DRIVETRAIN_VERTICAL_WHEEL_CENTER_DISTANCE;
            }
            else
            {
                // calculate distance to rotation point based on the first and second modules
                double tanSteeringAngle1 = Helpers.tand(-this.steerAngles[0]);
                double tanSteeringAngle2 = Helpers.tand(-this.steerAngles[1]);
                horizontalCenterDistance1 = (tanSteeringAngle1 + tanSteeringAngle2) * HardwareConstants.DRIVETRAIN_HORIZONTAL_WHEEL_CENTER_DISTANCE / (tanSteeringAngle1 - tanSteeringAngle2);
                verticalCenterDistance1 = (horizontalCenterDistance1 +  HardwareConstants.DRIVETRAIN_HORIZONTAL_WHEEL_CENTER_DISTANCE) * tanSteeringAngle2 - HardwareConstants.DRIVETRAIN_VERTICAL_WHEEL_CENTER_DISTANCE;
            }

            // calculate radius of first and second modules from the rotation point
            double r1 = Math.sqrt(Math.pow(horizontalCenterDistance1 - HardwareConstants.DRIVETRAIN_HORIZONTAL_WHEEL_CENTER_DISTANCE, 2.0) + Math.pow(HardwareConstants.DRIVETRAIN_VERTICAL_WHEEL_CENTER_DISTANCE - verticalCenterDistance1, 2.0));
            double r2 = Math.sqrt(Math.pow(horizontalCenterDistance1 + HardwareConstants.DRIVETRAIN_HORIZONTAL_WHEEL_CENTER_DISTANCE, 2.0) + Math.pow(HardwareConstants.DRIVETRAIN_VERTICAL_WHEEL_CENTER_DISTANCE - verticalCenterDistance1, 2.0));

            // calculate our turning speed based on drive motor velocity per radius for either the first or second module
            if (!Helpers.WithinDelta(r1, 0.0, 0.01))
            {
                omega1 = (this.driveVelocities[0] * HardwareConstants.DRIVETRAIN_DRIVE_VELOCITY_TO_INCHES_PER_SECOND) / r1;
                if (this.isDirectionSwapped[0])
                {
                    omega1 *= -1.0;
                }
            }
            else //if (!Helpers.WithinDelta(r2, 0.0, 0.01))
            {
                omega1 = (this.driveVelocities[1] * HardwareConstants.DRIVETRAIN_DRIVE_VELOCITY_TO_INCHES_PER_SECOND)/ r2;
                if (this.isDirectionSwapped[1])
                {
                    omega1 *= -1.0;
                }
            }
        }

        double omega2; // in radians / second
        double horizontalCenterDistance2;
        double verticalCenterDistance2;
        if (Helpers.AnglePairWithinDelta(this.steerAngles[2], this.isDirectionSwapped[2] ^ (this.driveVelocities[2] >= 0.0), this.steerAngles[3], this.isDirectionSwapped[3] ^ (this.driveVelocities[3] >= 0.0), 0.01))
        {
            omega2 = 0.0;
            horizontalCenterDistance2 = Double.POSITIVE_INFINITY;
            verticalCenterDistance2 = 0.0;
        }
        else
        {
            if (Helpers.WithinDelta(Math.abs(this.steerAngles[2]), 90.0, 0.005))
            {
                horizontalCenterDistance2 = -HardwareConstants.DRIVETRAIN_HORIZONTAL_WHEEL_CENTER_DISTANCE;
                verticalCenterDistance2 = HardwareConstants.DRIVETRAIN_VERTICAL_WHEEL_CENTER_DISTANCE - HardwareConstants.DRIVETRAIN_HORIZONTAL_WHEEL_SEPERATION_DISTANCE * Helpers.tand(-this.steerAngles[3]);
            }
            else if (Helpers.WithinDelta(Math.abs(this.steerAngles[3]), 90.0, 0.005))
            {
                horizontalCenterDistance2 = HardwareConstants.DRIVETRAIN_HORIZONTAL_WHEEL_CENTER_DISTANCE;
                verticalCenterDistance2 = HardwareConstants.DRIVETRAIN_VERTICAL_WHEEL_CENTER_DISTANCE - HardwareConstants.DRIVETRAIN_HORIZONTAL_WHEEL_SEPERATION_DISTANCE * Helpers.tand(this.steerAngles[2]);
            }
            else
            {
                // calculate distance to rotation point based on the third and fourth modules
                double tanSteeringAngle3 = Helpers.tand(this.steerAngles[2]);
                double tanSteeringAngle4 = Helpers.tand(this.steerAngles[3]);
                horizontalCenterDistance2 = (tanSteeringAngle3 + tanSteeringAngle4) * HardwareConstants.DRIVETRAIN_HORIZONTAL_WHEEL_CENTER_DISTANCE / (tanSteeringAngle4 - tanSteeringAngle3);
                verticalCenterDistance2 = HardwareConstants.DRIVETRAIN_VERTICAL_WHEEL_CENTER_DISTANCE - (horizontalCenterDistance2 - HardwareConstants.DRIVETRAIN_HORIZONTAL_WHEEL_CENTER_DISTANCE) * tanSteeringAngle4;
            }

            // calculate radius of third and fourth modules from the rotation point
            double r3 = Math.sqrt(Math.pow(horizontalCenterDistance2 + HardwareConstants.DRIVETRAIN_HORIZONTAL_WHEEL_CENTER_DISTANCE, 2.0) + Math.pow(HardwareConstants.DRIVETRAIN_VERTICAL_WHEEL_CENTER_DISTANCE + verticalCenterDistance2, 2.0));
            double r4 = Math.sqrt(Math.pow(horizontalCenterDistance2 - HardwareConstants.DRIVETRAIN_HORIZONTAL_WHEEL_CENTER_DISTANCE, 2.0) + Math.pow(HardwareConstants.DRIVETRAIN_VERTICAL_WHEEL_CENTER_DISTANCE + verticalCenterDistance2, 2.0));

            // calculate our turning speed based on drive motor velocity per radius for either the third or fourth module
            if (!Helpers.WithinDelta(r3, 0.0, 0.01))
            {
                omega2 = (this.driveVelocities[2] * HardwareConstants.DRIVETRAIN_DRIVE_VELOCITY_TO_INCHES_PER_SECOND) / r3;
                if (this.isDirectionSwapped[2])
                {
                    omega2 *= -1.0;
                }
            }
            else //if (!Helpers.WithinDelta(r4, 0.0, 0.01))
            {
                omega2 = (this.driveVelocities[3] * HardwareConstants.DRIVETRAIN_DRIVE_VELOCITY_TO_INCHES_PER_SECOND)/ r4;
                if (this.isDirectionSwapped[3])
                {
                    omega2 *= -1.0;
                }
            }
        }

        // average our center distances
        double averageHorizontalCenterDistance = (horizontalCenterDistance1 + horizontalCenterDistance2) / 2.0;
        double averageVerticalCenterDistance = (verticalCenterDistance1 + verticalCenterDistance2) / 2.0;

        // average our omegas, apply that to the angle
        double averageOmega = (omega1 + omega2) / 2.0;
        double horizontalVelocity;
        double verticalVelocity;
        if (Helpers.WithinDelta(averageOmega, 0.0, 0.01))
        {
            // this.angle doesn't change

            // calculate our horizontal and vertical velocities using an average of our various velocities and the angle.
            horizontalVelocity = 
                (Helpers.sind(this.steerAngles[0]) * HardwareConstants.DRIVETRAIN_DRIVE_VELOCITY_TO_INCHES_PER_SECOND * this.driveVelocities[0] * (this.isDirectionSwapped[0] ? -1.0 : 1.0) +
                    Helpers.sind(this.steerAngles[1]) * HardwareConstants.DRIVETRAIN_DRIVE_VELOCITY_TO_INCHES_PER_SECOND * this.driveVelocities[1] * (this.isDirectionSwapped[1] ? -1.0 : 1.0) +
                    Helpers.sind(this.steerAngles[2]) * HardwareConstants.DRIVETRAIN_DRIVE_VELOCITY_TO_INCHES_PER_SECOND * this.driveVelocities[2] * (this.isDirectionSwapped[2] ? -1.0 : 1.0) +
                    Helpers.sind(this.steerAngles[3]) * HardwareConstants.DRIVETRAIN_DRIVE_VELOCITY_TO_INCHES_PER_SECOND * this.driveVelocities[3] * (this.isDirectionSwapped[3] ? -1.0 : 1.0))
                / 4.0;

            verticalVelocity = 
                (Helpers.cosd(this.steerAngles[0]) * HardwareConstants.DRIVETRAIN_DRIVE_VELOCITY_TO_INCHES_PER_SECOND * this.driveVelocities[0] * (this.isDirectionSwapped[0] ? -1.0 : 1.0) +
                    Helpers.cosd(this.steerAngles[1]) * HardwareConstants.DRIVETRAIN_DRIVE_VELOCITY_TO_INCHES_PER_SECOND * this.driveVelocities[1] * (this.isDirectionSwapped[1] ? -1.0 : 1.0) +
                    Helpers.cosd(this.steerAngles[2]) * HardwareConstants.DRIVETRAIN_DRIVE_VELOCITY_TO_INCHES_PER_SECOND * this.driveVelocities[2] * (this.isDirectionSwapped[2] ? -1.0 : 1.0) +
                    Helpers.cosd(this.steerAngles[3]) * HardwareConstants.DRIVETRAIN_DRIVE_VELOCITY_TO_INCHES_PER_SECOND * this.driveVelocities[3] * (this.isDirectionSwapped[3] ? -1.0 : 1.0))
                / 4.0;
        }
        else
        {
            this.angle += averageOmega * this.deltaT;

            // calculate our horizontal and vertical velocities using the turn velocity...
            horizontalVelocity = averageVerticalCenterDistance * averageOmega;
            verticalVelocity = -averageHorizontalCenterDistance * averageOmega;
        }

        double horizontalCenterVelocity = horizontalVelocity * Helpers.cosd(this.angle) - verticalVelocity * Helpers.sind(this.angle);
        double verticalCenterVelocity = horizontalVelocity * Helpers.sind(this.angle) + verticalVelocity * Helpers.cosd(this.angle);
        this.xPosition += horizontalCenterVelocity * this.deltaT;
        this.yPosition += verticalCenterVelocity * this.deltaT;
    }

    private void assertPowerLevelRange(double powerLevel, String side)
    {
        if (powerLevel < DriveTrainMechanism.POWERLEVEL_MIN)
        {
            if (TuningConstants.THROW_EXCEPTIONS)
            {
                throw new RuntimeException(side + " power level too low!");
            }

            return;
        }

        if (powerLevel > DriveTrainMechanism.POWERLEVEL_MAX)
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
        return Helpers.EnforceRange(powerLevel, DriveTrainMechanism.POWERLEVEL_MIN, DriveTrainMechanism.POWERLEVEL_MAX);
    }

    private class Setpoint
    {
        private Double angle;
        private double drive;

        /**
         * Initializes a new Setpoint
         * @param drive value to apply
         * @param angle value to apply
         */
        public Setpoint(double drive, Double angle)
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
        public Double getAngle()
        {
            return this.angle;
        }
    }
}