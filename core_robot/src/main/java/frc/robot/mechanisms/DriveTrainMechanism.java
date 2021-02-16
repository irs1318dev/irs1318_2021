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

import javax.inject.Singleton;

import frc.robot.*;
import frc.robot.common.*;
import frc.robot.common.robotprovider.*;
import frc.robot.driver.*;
import frc.robot.driver.common.Driver;

import com.google.inject.Inject;

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

    private double time;
    private double xPosition;
    private double yPosition;

    private double robotYaw;
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

        double a1 = -HardwareConstants.DRIVETRAIN_HORIZONTAL_WHEEL_SEPERATION_DISTANCE / 2.0;
        double a2 = HardwareConstants.DRIVETRAIN_HORIZONTAL_WHEEL_SEPERATION_DISTANCE / 2.0;
        double b1 = -HardwareConstants.DRIVETRAIN_VERTICAL_WHEEL_SEPERATION_DISTANCE / 2.0;
        double b2 = HardwareConstants.DRIVETRAIN_VERTICAL_WHEEL_SEPERATION_DISTANCE / 2.0;

        this.MODULE_OFFSET_X = new double[] { a1, a2, a2, a1 };
        this.MODULE_OFFSET_Y = new double[] { b1, b1, b2, b2 };
        this.result = new Setpoint[DriveTrainMechanism.NUM_MODULES];

        this.time = 0.0;
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

        //double prevYaw = this.robotYaw;
        //double prevTime = this.time;
        this.robotYaw = this.navxManager.getAngle();
        this.time = this.timer.get();
        //double deltaT = this.time - prevTime;
        //double deltaYaw = (this.robotYaw - prevYaw) / deltaT;
    }

    public void update()
    {
        if (this.driver.getDigital(DigitalOperation.DriveTrainEnableFieldOrientation))
        {
            this.fieldOriented = true;
            this.desiredYaw = this.robotYaw;
        }

        if (this.driver.getDigital(DigitalOperation.DriveTrainDisableFieldOrientation) ||
            !this.navxManager.getIsConnected())
        {
            this.fieldOriented = false;
        }

        if (this.driver.getDigital(DigitalOperation.PositionResetFieldOrientation))
        {
            this.desiredYaw = this.robotYaw;
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

    private void calculateSetpoints()
    {
        // get the  center of rotation modifying control values
        double a = this.driver.getAnalog(AnalogOperation.DriveTrainRotationA);
        double b = this.driver.getAnalog(AnalogOperation.DriveTrainRotationB);

        // get the turning-related control values
        double turnX = this.driver.getAnalog(AnalogOperation.DriveTrainTurnX);
        double turnY = this.driver.getAnalog(AnalogOperation.DriveTrainTurnY);

        // get the center velocity control values
        double centerVelocityXRaw = this.driver.getAnalog(AnalogOperation.DriveTrainMoveSide);
        double centerVelocityYRaw = this.driver.getAnalog(AnalogOperation.DriveTrainMoveForward);

        // calculate center velocity and turn velocity based on our current control mode:
        double centerVelocityX;
        double centerVelocityY;
        double omega;
        if (this.fieldOriented)
        {
            centerVelocityX = Helpers.cosd(this.robotYaw) * centerVelocityXRaw + Helpers.sind(this.robotYaw) * centerVelocityYRaw;
            centerVelocityY = Helpers.cosd(this.robotYaw) * centerVelocityYRaw - Helpers.sind(this.robotYaw) * centerVelocityXRaw;

            if (TuningConstants.DRIVETRAIN_SKIP_ANGLE_ON_ZERO_VELOCITY
                && !Helpers.WithinDelta(Math.sqrt(turnX * turnX + turnY * turnY), 0.0, TuningConstants.DRIVETRAIN_SKIP_OMEGA_ON_ZERO_DELTA))
            {
                double steerGoal = Helpers.atan2d(-turnX, turnY);
                AnglePair anglePair = AnglePair.getClosestAngle(steerGoal, this.robotYaw, false);
                this.desiredYaw = anglePair.getAngle();
            }

            this.logger.logNumber(LoggingKey.DriveTrainDesiredAngle, this.desiredYaw);
            omega = -1.0 * this.omegaPID.calculatePosition(this.desiredYaw, this.robotYaw);
        }
        else
        {
            centerVelocityY = centerVelocityYRaw;
            centerVelocityX = centerVelocityXRaw;
            omega = turnX;
        }

        omega *= TuningConstants.DRIVETRAIN_TURN_VELOCITY;
        for (int i = 0; i < DriveTrainMechanism.NUM_MODULES; i++)
        {
            double moduleVelocityX = centerVelocityX - omega * (this.MODULE_OFFSET_Y[i] + b);
            double moduleVelocityY = centerVelocityY + omega * (this.MODULE_OFFSET_X[i] + a);

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