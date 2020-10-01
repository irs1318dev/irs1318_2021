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

    public boolean fieldOriented;

    private final ILogger logger;

    private Driver driver;

    private ITalonFX[] angleMotors;
    private ITalonFX[] driveMotors;
    private IAnalogInput[] absoluteEncoders;

    private double[] driveVelocities;
    private int[] drivePositions;
    private double[] driveErrors;
    private double[] angleVelocities;
    private int[] anglePositions;
    private double[] angleErrors;
    private double[] encoderVoltages;
    private double[] encoderAngles;

    private final LoggingKey[] encoderAnglesLK = { LoggingKey.DriveTrainAbsoluteEncoderPosition1, LoggingKey.DriveTrainAbsoluteEncoderPosition2, LoggingKey.DriveTrainAbsoluteEncoderPosition3, LoggingKey.DriveTrainAbsoluteEncoderPosition4 };
    private final LoggingKey[] driveVelocitiesLK = { LoggingKey.DriveTrainDriveVelocity1, LoggingKey.DriveTrainDriveVelocity2, LoggingKey.DriveTrainDriveVelocity3, LoggingKey.DriveTrainDriveVelocity4 };
    private final LoggingKey[] drivePositionsLK = { LoggingKey.DriveTrainDrivePosition1, LoggingKey.DriveTrainDrivePosition2, LoggingKey.DriveTrainDrivePosition3, LoggingKey.DriveTrainDrivePosition4 };
    private final LoggingKey[] driveErrorsLK = { LoggingKey.DriveTrainDriveError1, LoggingKey.DriveTrainDriveError2, LoggingKey.DriveTrainDriveError3, LoggingKey.DriveTrainDriveError4 };
    private final LoggingKey[] angleVelocitiesLK = { LoggingKey.DriveTrainAngleVelocity1, LoggingKey.DriveTrainAngleVelocity2, LoggingKey.DriveTrainAngleVelocity3, LoggingKey.DriveTrainAngleVelocity4 };
    private final LoggingKey[] anglePositionsLK = { LoggingKey.DriveTrainAnglePosition1, LoggingKey.DriveTrainAnglePosition2, LoggingKey.DriveTrainAnglePosition3, LoggingKey.DriveTrainAnglePosition4 };
    private final LoggingKey[] angleErrorsLK = { LoggingKey.DriveTrainAngleError1, LoggingKey.DriveTrainAngleError2, LoggingKey.DriveTrainAngleError3, LoggingKey.DriveTrainAngleError4 };
    private final LoggingKey[] driveGoalLK = { LoggingKey.DriveTrainDriveVelocityGoal1, LoggingKey.DriveTrainDriveVelocityGoal2, LoggingKey.DriveTrainDriveVelocityGoal3, LoggingKey.DriveTrainDriveVelocityGoal4 };
    private final LoggingKey[] angleGoalLK = { LoggingKey.DriveTrainAnglePositionGoal1, LoggingKey.DriveTrainAnglePositionGoal2, LoggingKey.DriveTrainAnglePositionGoal3, LoggingKey.DriveTrainAnglePositionGoal4 };

    @Inject
    public DriveTrainMechanism(
        LoggingManager logger,
        IRobotProvider provider)
    {
        this.logger = logger;

        this.angleMotors = new ITalonFX[4];
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

            this.angleMotors[i] = provider.getTalonFX(ElectronicsConstants.DRIVETRAIN_ANGLE_MOTOR_CAN_ID[i]);
            this.angleMotors[i].setInvertOutput(HardwareConstants.DRIVETRAIN_ANGLE_MOTOR_INVERT_OUTPUT[i]);
            this.angleMotors[i].setInvertSensor(HardwareConstants.DRIVETRAIN_ANGLE_MOTOR_INVERT_SENSOR[i]);
            this.angleMotors[i].setNeutralMode(MotorNeutralMode.Brake);
            this.angleMotors[i].setSensorType(TalonXFeedbackDevice.IntegratedSensor);
            this.angleMotors[i].setPIDF(
                TuningConstants.DRIVETRAIN_ANGLE_MOTOR_POSITION_PID_KP[i],
                TuningConstants.DRIVETRAIN_ANGLE_MOTOR_POSITION_PID_KI[i],
                TuningConstants.DRIVETRAIN_ANGLE_MOTOR_POSITION_PID_KD[i],
                TuningConstants.DRIVETRAIN_ANGLE_MOTOR_POSITION_PID_KF[i],
                DriveTrainMechanism.pidSlotId);

            this.angleMotors[i].setControlMode(TalonSRXControlMode.Position);

            this.absoluteEncoders[i] = provider.getAnalogInput(ElectronicsConstants.DRIVETRAIN_ABSOLUTE_ENCODER_ANALOG_INPUT[i]);
        }

        this.driveVelocities = new double[4];
        this.drivePositions = new int[4];
        this.driveErrors = new double[4];
        this.angleVelocities = new double[4];
        this.anglePositions = new int[4];
        this.angleErrors = new double[4];
        this.encoderVoltages = new double[4];
        this.encoderAngles = new double[4];
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
            this.angleVelocities[i] = this.angleMotors[i].getVelocity();
            this.anglePositions[i] = this.angleMotors[i].getPosition();
            this.angleErrors[i] = this.angleMotors[i].getError();
            this.encoderVoltages[i] = this.absoluteEncoders[i].getVoltage();
            this.encoderAngles[i] = this.encoderVoltages[i] * HardwareConstants.DRIVETRAIN_ENCODER_DEGREES_PER_VOLT;

            this.logger.logNumber(this.driveVelocitiesLK[i], this.driveVelocities[i]);
            this.logger.logNumber(this.drivePositionsLK[i], this.drivePositions[i]);
            this.logger.logNumber(this.driveErrorsLK[i], this.driveErrors[i]);
            this.logger.logNumber(this.angleVelocitiesLK[i], this.angleVelocities[i]);
            this.logger.logNumber(this.anglePositionsLK[i], this.anglePositions[i]);
            this.logger.logNumber(this.angleErrorsLK[i], this.angleErrors[i]);
            this.logger.logNumber(this.encoderAnglesLK[i], this.encoderAngles[i]);
        }
    }

    public void update()
    {
        Setpoint[] setpoints = this.calculateSetpoints();

        for (int i = 0; i < 4; i++)
        {
            Setpoint current = setpoints[i];
            Double angleSetpoint = current.getAngle();
            double driveSetpoint = current.getDrive();

            this.logger.logNumber(this.driveGoalLK[i], driveSetpoint);
            this.driveMotors[i].set(driveSetpoint);

            if (angleSetpoint != null)
            {
                this.logger.logNumber(this.angleGoalLK[i], angleSetpoint);
                this.angleMotors[i].set(angleSetpoint);
            }
        }

        if (this.driver.getDigital(DigitalOperation.DriveTrainReset))
        {
            for (int i = 0; i < 4; i++)
            {
                this.driveMotors[i].reset();
                this.angleMotors[i].reset();
            }
        }
        if (this.driver.getDigital(DigitalOperation.EnableFieldOrientation))
        {
            this.fieldOriented = true;
        }
        if (this.driver.getDigital(DigitalOperation.DisableFieldOrientation))
        {
            this.fieldOriented = false;
        }
    }

    public static double getClosestAngle(double desiredAngle, double currentAngle)
    {
        double multiplicand = Math.floor(currentAngle / 360.0);

        double[] closeRotations =
        {
            (desiredAngle + 360.0 * (multiplicand - 1.0)),
            (desiredAngle + 360.0 * multiplicand),
            (desiredAngle + 360.0 * (multiplicand + 1.0)),
        };

        double best = currentAngle;
        double bestDistance = Double.POSITIVE_INFINITY;
        for (int i = 0; i < 3; i++)
        {
            double angle = closeRotations[i];
            double angleDistance = Math.abs(currentAngle - angle);
            if (angleDistance < bestDistance)
            {
                best = angle;
                bestDistance = angleDistance;
            }
        }

        return best;
    }

    public void stop()
    {
        for (int i = 0; i < 4; i++)
        {
            this.driveMotors[i].stop();
            this.angleMotors[i].stop();
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
        // double turnY = this.driver.getAnalog(AnalogOperation.DriveTrainTurnY);
        double Vcy = this.driver.getAnalog(AnalogOperation.DriveTrainMoveForward);
        double Vcx = this.driver.getAnalog(AnalogOperation.DriveTrainMoveSide);

        double omega = turnX * TuningConstants.DRIVETRAIN_TURN_VELOCITY;

        for (int i = 0; i < 4; i++)
        {
            double Vx = Vcx - omega * Ry[i];
            double Vy = Vcy + omega * Rx[i]; // quik mafs

            Double anglePositionGoal;
            double driveVelocityGoal = Math.sqrt(Vx * Vx + Vy * Vy);
            if (TuningConstants.DRIVETRAIN_SKIP_ANGLE_ON_ZERO_VELOCITY &&
                Helpers.WithinDelta(Vx, 0.0, TuningConstants.DRIVETRAIN_SKIP_ANGLE_ON_ZERO_DELTA) &&
                Helpers.WithinDelta(Vy, 0.0, TuningConstants.DRIVETRAIN_SKIP_ANGLE_ON_ZERO_DELTA))
            {
                anglePositionGoal = null;
            }
            else
            {
                anglePositionGoal = Helpers.EnforceRange(Helpers.atan2d(-Vx, Vy), -180.0, 180.0);
                double currentAngle = this.anglePositions[i] / TuningConstants.DRIVETRAIN_ANGLE_MOTOR_POSITION_PID_KS;
                anglePositionGoal = DriveTrainMechanism.getClosestAngle(anglePositionGoal, currentAngle);    
                anglePositionGoal *= TuningConstants.DRIVETRAIN_ANGLE_MOTOR_POSITION_PID_KS;
            }

            driveVelocityGoal = this.applyPowerLevelRange(driveVelocityGoal);

            this.assertPowerLevelRange(driveVelocityGoal, "drive");

            driveVelocityGoal *= TuningConstants.DRIVETRAIN_DRIVE_MOTOR_VELOCITY_PID_KS;

            result[i] = new Setpoint(driveVelocityGoal, anglePositionGoal);
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