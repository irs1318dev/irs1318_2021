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
    private static final int pidSlotId = 0;
    private static final int FRAME_PERIOD_MS = 5;

    private static final double POWERLEVEL_MIN = -1.0;
    private static final double POWERLEVEL_MAX = 1.0;

    private final PositionManager positionManager;
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

    private double robotYaw;
    private double[] driveVelocities;
    private int[] drivePositions;
    private double[] driveErrors;
    private double[] steerVelocities;
    private int[] steerPositions;
    private double[] steerAngles;
    private double[] steerErrors;
    private double[] encoderVoltages;
    private double[] encoderAngles;

    private final LoggingKey[] encoderAnglesLK = { LoggingKey.DriveTrainAbsoluteEncoderAngle1, LoggingKey.DriveTrainAbsoluteEncoderAngle2, LoggingKey.DriveTrainAbsoluteEncoderAngle3, LoggingKey.DriveTrainAbsoluteEncoderAngle4 };
    private final LoggingKey[] driveVelocitiesLK = { LoggingKey.DriveTrainDriveVelocity1, LoggingKey.DriveTrainDriveVelocity2, LoggingKey.DriveTrainDriveVelocity3, LoggingKey.DriveTrainDriveVelocity4 };
    private final LoggingKey[] drivePositionsLK = { LoggingKey.DriveTrainDrivePosition1, LoggingKey.DriveTrainDrivePosition2, LoggingKey.DriveTrainDrivePosition3, LoggingKey.DriveTrainDrivePosition4 };
    private final LoggingKey[] driveErrorsLK = { LoggingKey.DriveTrainDriveError1, LoggingKey.DriveTrainDriveError2, LoggingKey.DriveTrainDriveError3, LoggingKey.DriveTrainDriveError4 };
    private final LoggingKey[] steerVelocitiesLK = { LoggingKey.DriveTrainSteerVelocity1, LoggingKey.DriveTrainSteerVelocity2, LoggingKey.DriveTrainSteerVelocity3, LoggingKey.DriveTrainSteerVelocity4 };
    private final LoggingKey[] steerPositionsLK = { LoggingKey.DriveTrainSteerPosition1, LoggingKey.DriveTrainSteerPosition2, LoggingKey.DriveTrainSteerPosition3, LoggingKey.DriveTrainSteerPosition4 };
    private final LoggingKey[] steerAnglesLK = { LoggingKey.DriveTrainSteerAngle1, LoggingKey.DriveTrainSteerAngle2, LoggingKey.DriveTrainSteerAngle3, LoggingKey.DriveTrainSteerAngle4 };
    private final LoggingKey[] steerErrorsLK = { LoggingKey.DriveTrainSteerError1, LoggingKey.DriveTrainSteerError2, LoggingKey.DriveTrainSteerError3, LoggingKey.DriveTrainSteerError4 };
    private final LoggingKey[] driveGoalLK = { LoggingKey.DriveTrainDriveVelocityGoal1, LoggingKey.DriveTrainDriveVelocityGoal2, LoggingKey.DriveTrainDriveVelocityGoal3, LoggingKey.DriveTrainDriveVelocityGoal4 };
    private final LoggingKey[] steerGoalLK = { LoggingKey.DriveTrainSteerPositionGoal1, LoggingKey.DriveTrainSteerPositionGoal2, LoggingKey.DriveTrainSteerPositionGoal3, LoggingKey.DriveTrainSteerPositionGoal4 };

    @Inject
    public DriveTrainMechanism(
        LoggingManager logger,
        IRobotProvider provider,
        PositionManager positionManager,
        ITimer timer)
    {
        this.timer = timer;
        this.logger = logger;
        this.positionManager = positionManager;

        this.steerMotors = new ITalonFX[4];
        this.driveMotors = new ITalonFX[4];
        this.absoluteEncoders = new IAnalogInput[4];
        for (int i = 0; i <= 3; i++)
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

        this.driveVelocities = new double[4];
        this.drivePositions = new int[4];
        this.driveErrors = new double[4];
        this.steerVelocities = new double[4];
        this.steerPositions = new int[4];
        this.steerAngles = new double[4];
        this.steerErrors = new double[4];
        this.encoderVoltages = new double[4];
        this.encoderAngles = new double[4];

        this.isDirectionSwapped = new boolean[4];

        this.omegaPID = new PIDHandler(
            TuningConstants.DRIVETRAIN_OMEGA_POSITION_PID_KP,
            TuningConstants.DRIVETRAIN_OMEGA_POSITION_PID_KI,
            TuningConstants.DRIVETRAIN_OMEGA_POSITION_PID_KD,
            TuningConstants.DRIVETRAIN_OMEGA_POSITION_PID_KF,
            TuningConstants.DRIVETRAIN_OMEGA_POSITION_PID_KS,
            TuningConstants.DRIVETRAIN_OMEGA_MIN_OUTPUT,
            TuningConstants.DRIVETRAIN_OMEGA_MAX_OUTPUT,
            this.timer);
    }

    @Override
    public void setDriver(Driver driver)
    {
        this.driver = driver;
    }

    @Override
    public void readSensors()
    {
        for (int i = 0; i < 4; i++)
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

            this.logger.logNumber(this.driveVelocitiesLK[i], this.driveVelocities[i]);
            this.logger.logNumber(this.drivePositionsLK[i], this.drivePositions[i]);
            this.logger.logNumber(this.driveErrorsLK[i], this.driveErrors[i]);
            this.logger.logNumber(this.steerVelocitiesLK[i], this.steerVelocities[i]);
            this.logger.logNumber(this.steerPositionsLK[i], this.steerPositions[i]);
            this.logger.logNumber(this.steerAnglesLK[i], this.steerAngles[i]);
            this.logger.logNumber(this.steerErrorsLK[i], this.steerErrors[i]);
            this.logger.logNumber(this.encoderAnglesLK[i], this.encoderAngles[i]);
        }
    }

    public void update()
    {
        this.robotYaw = this.positionManager.getNavxAngle();

        if (this.driver.getDigital(DigitalOperation.DriveTrainEnableFieldOrientation))
        {
            this.fieldOriented = true;
            this.desiredYaw = this.robotYaw;
        }

        if (this.driver.getDigital(DigitalOperation.DriveTrainDisableFieldOrientation) ||
            !this.positionManager.getNavxIsConnected())
        {
            this.fieldOriented = false;
        }

        if (this.driver.getDigital(DigitalOperation.PositionResetFieldOrientation))
        {
            this.desiredYaw = this.robotYaw;
        }

        if (this.driver.getDigital(DigitalOperation.DriveTrainReset))
        {
            for (int i = 0; i < 4; i++)
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

        Setpoint[] setpoints = this.calculateSetpoints();
        for (int i = 0; i < 4; i++)
        {
            Setpoint current = setpoints[i];
            Double steerSetpoint = current.getAngle();
            double driveSetpoint = current.getDrive();

            this.logger.logNumber(this.driveGoalLK[i], driveSetpoint);
            this.driveMotors[i].set(driveSetpoint);

            if (steerSetpoint != null)
            {
                this.logger.logNumber(this.steerGoalLK[i], steerSetpoint);
                this.steerMotors[i].set(steerSetpoint);
            }
        }
    }

    public void stop()
    {
        this.omegaPID.reset();
        for (int i = 0; i < 4; i++)
        {
            this.driveMotors[i].stop();
            this.steerMotors[i].stop();
        }
    }

    private Setpoint[] calculateSetpoints()
    {
        double a = this.driver.getAnalog(AnalogOperation.DriveTrainRotationA); // center of rotation set to center of robot for now
        double b = this.driver.getAnalog(AnalogOperation.DriveTrainRotationB);

        Setpoint[] result = new Setpoint[4];

        double a1 = a - HardwareConstants.DRIVETRAIN_HORIZONTAL_WHEEL_SEPERATION_DISTANCE / 2.0;
        double a2 = a + HardwareConstants.DRIVETRAIN_HORIZONTAL_WHEEL_SEPERATION_DISTANCE / 2.0;
        double b1 = b - HardwareConstants.DRIVETRAIN_VERTICAL_WHEEL_SEPERATION_DISTANCE / 2.0;
        double b2 = b + HardwareConstants.DRIVETRAIN_VERTICAL_WHEEL_SEPERATION_DISTANCE / 2.0;

        double[] Rx = { a1, a2, a2, a1 };
        double[] Ry = { b1, b1, b2, b2 };

        double turnX = this.driver.getAnalog(AnalogOperation.DriveTrainTurnX);
        double turnY = this.driver.getAnalog(AnalogOperation.DriveTrainTurnY);
        double Vcy_raw = this.driver.getAnalog(AnalogOperation.DriveTrainMoveForward);
        double Vcx_raw = this.driver.getAnalog(AnalogOperation.DriveTrainMoveSide);

        double Vcy;
        double Vcx;
        double omega;
        if (this.fieldOriented)
        {
            Vcx = Helpers.cosd(this.robotYaw) * Vcx_raw + Helpers.sind(this.robotYaw) * Vcy_raw;
            Vcy = Helpers.cosd(this.robotYaw) * Vcy_raw - Helpers.sind(this.robotYaw) * Vcx_raw;

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
            Vcy = Vcy_raw;
            Vcx = Vcx_raw;
            omega = turnX;
        }

        omega *= TuningConstants.DRIVETRAIN_TURN_VELOCITY;
        for (int i = 0; i < 4; i++)
        {
            double Vx = Vcx - omega * Ry[i]; // quik mafs
            double Vy = Vcy + omega * Rx[i]; // skidy drop pop pop

            Double steerPositionGoal;
            double driveVelocityGoal;
            if (TuningConstants.DRIVETRAIN_SKIP_ANGLE_ON_ZERO_VELOCITY
                    && Helpers.WithinDelta(Vx, 0.0, TuningConstants.DRIVETRAIN_SKIP_ANGLE_ON_ZERO_DELTA)
                    && Helpers.WithinDelta(Vy, 0.0, TuningConstants.DRIVETRAIN_SKIP_ANGLE_ON_ZERO_DELTA))
            {
                driveVelocityGoal = 0.0;
                steerPositionGoal = null;
            }
            else
            {
                driveVelocityGoal = Math.sqrt(Vx * Vx + Vy * Vy);

                steerPositionGoal = Helpers.EnforceRange(Helpers.atan2d(-Vx, Vy), -180.0, 180.0);
                double currentAngle = this.steerPositions[i] * HardwareConstants.DRIVETRAIN_STEER_PULSE_DISTANCE;
                AnglePair anglePair = AnglePair.getClosestAngle(steerPositionGoal, currentAngle, true);
                steerPositionGoal = anglePair.getAngle() * TuningConstants.DRIVETRAIN_STEER_MOTOR_POSITION_PID_KS;
                this.isDirectionSwapped[i] = anglePair.getSwapDirection();
            }

            driveVelocityGoal = this.applyPowerLevelRange(driveVelocityGoal);

            this.assertPowerLevelRange(driveVelocityGoal, "drive");

            driveVelocityGoal *= TuningConstants.DRIVETRAIN_DRIVE_MOTOR_VELOCITY_PID_KS;
            if (this.isDirectionSwapped[i])
            {
                driveVelocityGoal *= -1.0;
            }

            result[i] = new Setpoint(driveVelocityGoal, steerPositionGoal);
        }

        return result;
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