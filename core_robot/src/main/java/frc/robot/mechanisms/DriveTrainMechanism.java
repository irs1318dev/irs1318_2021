/*
 * DriveTrainMechanism
 * 
 * authors: Will, Vanshika, Arushi
 * 
 * Started idk sometime in september
 * 
 * dO yOu ReMeMbEr 
 * tHe 21sT nIgHt oF sEpTeMbEr
*/

package frc.robot.mechanisms;

import java.util.ArrayList;
import java.util.BitSet;
import java.util.List;

import javax.inject.Singleton;

import frc.robot.*;
import frc.robot.common.*;
import frc.robot.common.robotprovider.*;
import frc.robot.driver.*;
import frc.robot.driver.common.Driver;

import com.google.common.collect.Range;
import com.google.inject.Inject;

@Singleton
public class DriveTrainMechanism implements IMechanism
{
    private static final int pidSlotId = 0;
    private static final int FRAME_PERIOD_MS = 5;

    private static final double POWERLEVEL_MIN = -1.0;
    private static final double POWERLEVEL_MAX = 1.0;

    private final ILogger logger;

    private final IAnalogInput absoluteEncoder1;
    private final IAnalogInput absoluteEncoder2;
    private final IAnalogInput absoluteEncoder3;
    private final IAnalogInput absoluteEncoder4;

    private final ITalonFX driveMotor1;
    private final ITalonFX angleMotor1;
    private final ITalonFX driveMotor2;
    private final ITalonFX angleMotor2;
    private final ITalonFX driveMotor3;
    private final ITalonFX angleMotor3;
    private final ITalonFX driveMotor4;
    private final ITalonFX angleMotor4;

    private Driver driver;

    private double encoderAngle1;
    private double encoderAngle2;
    private double encoderAngle3;
    private double encoderAngle4;

    private double angleError1;
    private double angleError2;
    private double angleError3;
    private double angleError4;

    private double driveError1;
    private double driveError2;
    private double driveError3;
    private double driveError4;

    private double driveVelocity1;
    private double driveVelocity2;
    private double driveVelocity3;
    private double driveVelocity4;

    private int drivePosition1;
    private int drivePosition2;
    private int drivePosition3;
    private int drivePosition4;

    private double angleVelocity1;
    private double angleVelocity2;
    private double angleVelocity3;
    private double angleVelocity4;

    private int anglePosition1;
    private int anglePosition2;
    private int anglePosition3;
    private int anglePosition4;

    private ITalonFX[] angleMotors = {this.angleMotor1, this.angleMotor2, this.angleMotor3, this.angleMotor4};
    private ITalonFX[] driveMotors = {this.driveMotor1, this.driveMotor2, this.driveMotor3, this.driveMotor4};
    private double[] encoderVoltages;
    private double[] encoderAngles = {this.encoderAngle1, this.encoderAngle2, this.encoderAngle3, this.encoderAngle4};
    private double[] driveVelocities = {this.driveVelocity1, this.driveVelocity2, this.driveVelocity3, this.encoderAngle4};
    private int[] drivePositions = {this.drivePosition1, this.drivePosition2, this.drivePosition3, this.drivePosition4};
    private double[] driveErrors = {this.driveError1, this.driveError2, this.driveError3, this.driveError4};
    private double[] angleVelocities = {this.angleVelocity1, this.angleVelocity2, this.angleVelocity3, this.angleVelocity4};
    private int[] anglePositions = {this.anglePosition1, this.anglePosition2, this.anglePosition3, this.anglePosition4};;
    private double[] angleErrors = {this.angleError1, this.angleError2, this.angleError3, this.angleError4};
    private IAnalogInput[] absoluteEncoders = {this.absoluteEncoder1, this.absoluteEncoder2, this.absoluteEncoder3, this.absoluteEncoder4};

    private LoggingKey[] encoderAnglesLK = {LoggingKey.DriveTrainAbsoluteEncoderPosition1, LoggingKey.DriveTrainAbsoluteEncoderPosition2, LoggingKey.DriveTrainAbsoluteEncoderPosition3, LoggingKey.DriveTrainAbsoluteEncoderPosition4};
    private LoggingKey[] driveVelocitiesLK = {LoggingKey.DriveTrainDriveVelocity1, LoggingKey.DriveTrainDriveVelocity2, LoggingKey.DriveTrainDriveVelocity3, LoggingKey.DriveTrainDriveVelocity};
    private LoggingKey[] drivePositionsLK = {LoggingKey.DriveTrainDrivePosition1, LoggingKey.DriveTrainAnglePosition1, LoggingKey.DriveTrainDrivePosition2, LoggingKey.DriveTrainAnglePosition2};
    private LoggingKey[] driveErrorsLK = {LoggingKey.DriveTrainDriveError1, LoggingKey.DriveTrainDriveError2, LoggingKey.DriveTrainDriveError3, LoggingKey.DriveTrainDriveError4,};
    private LoggingKey[] angleVelocitiesLK = {LoggingKey.DriveTrainAngleVelocity1, LoggingKey.DriveTrainAngleVelocity2, LoggingKey.DriveTrainAngleVelocity3, LoggingKey.DriveTrainAngleVelocity4,};
    private LoggingKey[] anglePositionsLK = {LoggingKey.DriveTrainDrivePosition3, LoggingKey.DriveTrainAnglePosition3, LoggingKey.DriveTrainDrivePosition4, LoggingKey.DriveTrainAnglePosition4};;
    private LoggingKey[] angleErrorsLK = {LoggingKey.DriveTrainAngleError1, LoggingKey.DriveTrainAngleError2, LoggingKey.DriveTrainAngleError3, LoggingKey.DriveTrainAngleError4,};

    @Inject
    public DriveTrainMechanism(
        LoggingManager logger,
        IRobotProvider provider)
    {
        this.logger = logger;
        
        //MODULE 1

        for(i = 0; i <=3; i++){
            this.driveMotors[i].setNeutralMode(MotorNeutralMode.Brake); //
            this.driveMotors[i].setSensorType(TalonXFeedbackDevice.IntegratedSensor); //
            this.driveMotors[i].setFeedbackFramePeriod(DriveTrainMechanism.FRAME_PERIOD_MS); //
            this.driveMotors[i].setPIDFFramePeriod(DriveTrainMechanism.FRAME_PERIOD_MS); //

            this.driveMotors[i].setVoltageCompensation(
                TuningConstants.DRIVETRAIN_VOLTAGE_COMPENSATION_ENABLED,
                TuningConstants.DRIVETRAIN_VOLTAGE_COMPENSATION);
            this.driveMotors[i].setSupplyCurrentLimit(
                TuningConstants.DRIVETRAIN_SUPPLY_CURRENT_LIMITING_ENABLED,
                TuningConstants.DRIVETRAIN_SUPPLY_CURRENT_MAX,
                TuningConstants.DRIVETRAIN_SUPPLY_TRIGGER_CURRENT,
                TuningConstants.DRIVETRAIN_SUPPLY_TRIGGER_DURATION);

            this.angleMotorS[i].setControlMode(TalonSRXControlMode.Position);
            this.driveMotors[i].setControlMode(TalonSRXControlMode.Velocity);

        }
        //MODULE 1
        this.angleMotor1 = provider.getTalonFX(ElectronicsConstants.DRIVETRAIN_ANGLE_MOTOR_1_CAN_ID);
        this.angleMotor1.setInvertOutput(HardwareConstants.DRIVETRAIN_ANGLE_MOTOR_1_INVERT_OUTPUT);
        this.angleMotor1.setInvertSensor(HardwareConstants.DRIVETRAIN_ANGLE_MOTOR_1_INVERT_SENSOR);
        this.angleMotor1.setNeutralMode(MotorNeutralMode.Brake);
        this.angleMotor1.setSensorType(TalonXFeedbackDevice.IntegratedSensor);
        //this.angleMotor.setPosition((int)this.encoderAngle); //would this work?
        this.angleMotor1.setPIDF(
            TuningConstants.DRIVETRAIN_ANGLE_MOTOR_1_POSITION_PID_KP,
            TuningConstants.DRIVETRAIN_ANGLE_MOTOR_1_POSITION_PID_KI,
            TuningConstants.DRIVETRAIN_ANGLE_MOTOR_1_POSITION_PID_KD,
            TuningConstants.DRIVETRAIN_ANGLE_MOTOR_1_POSITION_PID_KF,
            DriveTrainMechanism.pidSlotId);

        //MODULE 2
        this.angleMotor2 = provider.getTalonFX(ElectronicsConstants.DRIVETRAIN_ANGLE_MOTOR_2_CAN_ID);
        this.angleMotor2.setInvertOutput(HardwareConstants.DRIVETRAIN_ANGLE_MOTOR_2_INVERT_OUTPUT);
        this.angleMotor2.setInvertSensor(HardwareConstants.DRIVETRAIN_ANGLE_MOTOR_2_INVERT_SENSOR);
        this.angleMotor2.setNeutralMode(MotorNeutralMode.Brake);
        this.angleMotor2.setSensorType(TalonXFeedbackDevice.IntegratedSensor);
        this.angleMotor2.setPIDF(
            TuningConstants.DRIVETRAIN_ANGLE_MOTOR_2_POSITION_PID_KP,
            TuningConstants.DRIVETRAIN_ANGLE_MOTOR_2_POSITION_PID_KI,
            TuningConstants.DRIVETRAIN_ANGLE_MOTOR_2_POSITION_PID_KD,
            TuningConstants.DRIVETRAIN_ANGLE_MOTOR_2_POSITION_PID_KF,
            DriveTrainMechanism.pidSlotId);

        this.driveMotor2 = provider.getTalonFX(ElectronicsConstants.DRIVETRAIN_DRIVE_MOTOR_2_CAN_ID);
        this.driveMotor2.setInvertOutput(HardwareConstants.DRIVETRAIN_DRIVE_MOTOR_2_INVERT_OUTPUT);
        this.driveMotor2.setInvertSensor(HardwareConstants.DRIVETRAIN_DRIVE_MOTOR_2_INVERT_SENSOR);
        this.driveMotor2.configureVelocityMeasurements(10, 32); //
        this.driveMotor2.setPIDF(
            TuningConstants.DRIVETRAIN_DRIVE_MOTOR_2_VELOCITY_PID_KP,
            TuningConstants.DRIVETRAIN_DRIVE_MOTOR_2_VELOCITY_PID_KI,
            TuningConstants.DRIVETRAIN_DRIVE_MOTOR_2_VELOCITY_PID_KD,
            TuningConstants.DRIVETRAIN_DRIVE_MOTOR_2_VELOCITY_PID_KF,
            DriveTrainMechanism.pidSlotId);


        //MODULE 3
        this.angleMotor3 = provider.getTalonFX(ElectronicsConstants.DRIVETRAIN_ANGLE_MOTOR_3_CAN_ID);
        this.angleMotor3.setInvertOutput(HardwareConstants.DRIVETRAIN_ANGLE_MOTOR_3_INVERT_OUTPUT);
        this.angleMotor3.setInvertSensor(HardwareConstants.DRIVETRAIN_ANGLE_MOTOR_3_INVERT_SENSOR);
        this.angleMotor3.setNeutralMode(MotorNeutralMode.Brake);
        this.angleMotor3.setSensorType(TalonXFeedbackDevice.IntegratedSensor);
        this.angleMotor3.setPIDF(
            TuningConstants.DRIVETRAIN_ANGLE_MOTOR_3_POSITION_PID_KP,
            TuningConstants.DRIVETRAIN_ANGLE_MOTOR_3_POSITION_PID_KI,
            TuningConstants.DRIVETRAIN_ANGLE_MOTOR_3_POSITION_PID_KD,
            TuningConstants.DRIVETRAIN_ANGLE_MOTOR_3_POSITION_PID_KF,
            DriveTrainMechanism.pidSlotId);

        this.driveMotor3 = provider.getTalonFX(ElectronicsConstants.DRIVETRAIN_DRIVE_MOTOR_3_CAN_ID);
        this.driveMotor3.setInvertOutput(HardwareConstants.DRIVETRAIN_DRIVE_MOTOR_3_INVERT_OUTPUT);
        this.driveMotor3.setInvertSensor(HardwareConstants.DRIVETRAIN_DRIVE_MOTOR_3_INVERT_SENSOR);
        this.driveMotor3.configureVelocityMeasurements(10, 32); //
        this.driveMotor3.setPIDF(
            TuningConstants.DRIVETRAIN_DRIVE_MOTOR_3_VELOCITY_PID_KP,
            TuningConstants.DRIVETRAIN_DRIVE_MOTOR_3_VELOCITY_PID_KI,
            TuningConstants.DRIVETRAIN_DRIVE_MOTOR_3_VELOCITY_PID_KD,
            TuningConstants.DRIVETRAIN_DRIVE_MOTOR_3_VELOCITY_PID_KF,
            DriveTrainMechanism.pidSlotId);

        //MODULE 4
        this.angleMotor4 = provider.getTalonFX(ElectronicsConstants.DRIVETRAIN_ANGLE_MOTOR_4_CAN_ID);
        this.angleMotor4.setInvertOutput(HardwareConstants.DRIVETRAIN_ANGLE_MOTOR_4_INVERT_OUTPUT);
        this.angleMotor4.setInvertSensor(HardwareConstants.DRIVETRAIN_ANGLE_MOTOR_4_INVERT_SENSOR);
        this.angleMotor4.setNeutralMode(MotorNeutralMode.Brake);
        this.angleMotor4.setSensorType(TalonXFeedbackDevice.IntegratedSensor);
        //this.angleMotor.setPosition((int)this.encoderAngle); //would this work?
        this.angleMotor4.setPIDF(
            TuningConstants.DRIVETRAIN_ANGLE_MOTOR_4_POSITION_PID_KP,
            TuningConstants.DRIVETRAIN_ANGLE_MOTOR_4_POSITION_PID_KI,
            TuningConstants.DRIVETRAIN_ANGLE_MOTOR_4_POSITION_PID_KD,
            TuningConstants.DRIVETRAIN_ANGLE_MOTOR_4_POSITION_PID_KF,
            DriveTrainMechanism.pidSlotId);

        this.driveMotor4 = provider.getTalonFX(ElectronicsConstants.DRIVETRAIN_DRIVE_MOTOR_4_CAN_ID);
        this.driveMotor4.setInvertOutput(HardwareConstants.DRIVETRAIN_DRIVE_MOTOR_4_INVERT_OUTPUT);
        this.driveMotor4.setInvertSensor(HardwareConstants.DRIVETRAIN_DRIVE_MOTOR_4_INVERT_SENSOR);
        this.driveMotor4.configureVelocityMeasurements(10, 32); //WHAT IS THIS?
        this.driveMotor4.setPIDF(
            TuningConstants.DRIVETRAIN_DRIVE_MOTOR_4_VELOCITY_PID_KP,
            TuningConstants.DRIVETRAIN_DRIVE_MOTOR_4_VELOCITY_PID_KI,
            TuningConstants.DRIVETRAIN_DRIVE_MOTOR_4_VELOCITY_PID_KD,
            TuningConstants.DRIVETRAIN_DRIVE_MOTOR_4_VELOCITY_PID_KF,
            DriveTrainMechanism.pidSlotId);

        this.absoluteEncoder1 = provider.getAnalogInput(ElectronicsConstants.DRIVETRAIN_ABSOLUTE_ENCODER_1_ANALOG_INPUT);        
        this.absoluteEncoder2 = provider.getAnalogInput(ElectronicsConstants.DRIVETRAIN_ABSOLUTE_ENCODER_2_ANALOG_INPUT);
        this.absoluteEncoder3 = provider.getAnalogInput(ElectronicsConstants.DRIVETRAIN_ABSOLUTE_ENCODER_3_ANALOG_INPUT);
        this.absoluteEncoder4 = provider.getAnalogInput(ElectronicsConstants.DRIVETRAIN_ABSOLUTE_ENCODER_4_ANALOG_INPUT);

    }

    @Override
    public void setDriver(Driver driver)
    {
        this.driver = driver;
    }

    @Override
    public void readSensors()
    {
        this.encoderVoltages = new double[4]; 
        for(int i = 0; i <=3; i++){
            this.encoderVoltages[i] = this.absoluteEncoders[i].getVoltage();
            this.encoderAngles[i] = this.encoderVoltages[i] * HardwareConstants.DRIVETRAIN_ENCODER_DEGREES_PER_VOLT;
            this.driveVelocities[i] = this.driveMotors[i].getVelocity();
            this.drivePositions[i] = this.driveMotors[i].getPosition();
            this.driveErrors[i] = this.driveMotors[i].getError();
            this.angleVelocities[i] = this.angleMotors[i].getVelocity();
            this.anglePositions[i] = this.angleMotors[i].getPosition();
            this.angleErrors[i] = this.angleMotors[i].getError();
            this.logger.logNumber(this.driveVelocitiesLK[i], this.driveVelocities[i]);
            this.logger.logNumber(this.driveErrorsLK[i], this.driveErrors[i]);
            this.logger.logNumber(this.angleVelocitiesLK[i], this.angleVelocities[i]);
            this.logger.logNumber(this.drivePositionsLK[i], this.anglePositions[i]);
            this.logger.logNumber(this.anglePositionsLK[i], this.drivePositions[i]);
            this.logger.logNumber(this.encoderAnglesLK[i], this.encoderAngles[i]);
        }
    }

    public void update()
    {
        List<Setpoint> setpoint = this.calculateSetpoint();
        ITalonFX[] angleMotors = {this.angleMotor1, this.angleMotor2, this.angleMotor3, this.angleMotor4};
        ITalonFX[] driveMotors = {this.driveMotor1, this.driveMotor2, this.driveMotor3, this.driveMotor4};
        
        LoggingKey[] driveLogs = {LoggingKey.DriveTrainDriveVelocityGoal1, LoggingKey.DriveTrainDriveVelocityGoal2, LoggingKey.DriveTrainDriveVelocityGoal3, LoggingKey.DriveTrainDriveVelocityGoal4};
        LoggingKey[] angleLogs = {LoggingKey.DriveTrainAnglePositionGoal1, LoggingKey.DriveTrainAnglePositionGoal2, LoggingKey.DriveTrainAnglePositionGoal3, LoggingKey.DriveTrainAnglePositionGoal4};

        for(int i = 0; i < 4; i++)
        {
            Setpoint current = setpoint.get(i);
            double angleSetpoint = getClosestAngleInRange(
                current.getAngle(), 
                angleMotors[i].getPosition());
            double driveSetpoint = current.getDrive();

            this.logger.logNumber(angleLogs[i], angleSetpoint);
            this.logger.logNumber(driveLogs[i], driveSetpoint);

            driveMotors[i].set(driveSetpoint);
            angleMotors[i].set(angleSetpoint);
        }

        if (this.driver.getDigital(DigitalOperation.DriveTrainReset))
        {
            for(int i = 0; i < 4; i++)
            {
                driveMotors[i].reset();
                angleMotors[i].reset();
            }
        }
    }

    private double getClosestAngleInRange(double desiredAngle, double currentAngle)
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
            if (Helpers.WithinRange(angle, minRangeValue, maxRangeValue))
            {
                double angleDistance = Math.abs(currentAngle - angle);
                if (angleDistance < bestDistance)
                {
                    best = angle;
                    bestDistance = angleDistance;
                }
            }
        }
        return best;
    }

    public void stop()
    {
        for(int i = 0; i < 4; i++)
        {
            driveMotors[i].stop();
            angleMotors[i].stop();
        }
    }


    private class Setpoint
    {
        private double angle;
        private double drive;


         /* Initializes a new Setpoint
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


    private List<Setpoint> calculateSetpoint()
    {
        double a = 0.0; // center of rotation set to center of robot for now
        double b = 0.0;

        List<Setpoint> result = new ArrayList<>();
        
        double a1 = a - HardwareConstants.DRIVETRAIN_HORIZONTAL_WHEEL_SEPERATION_DISTANCE/2;
        double a2 = a + HardwareConstants.DRIVETRAIN_HORIZONTAL_WHEEL_SEPERATION_DISTANCE/2;
        double b1 = b - HardwareConstants.DRIVETRAIN_VERTICAL_WHEEL_SEPERATION_DISTANCE/2;
        double b2 = b + HardwareConstants.DRIVETRAIN_VERTICAL_WHEEL_SEPERATION_DISTANCE/2;

        double[] Rx = {a1, a2, a2, a1}; 
        double[] Ry = {b1, b1, b2, b2};

        double turnX = this.driver.getAnalog(AnalogOperation.DriveTrainTurnX);
        double turnY = this.driver.getAnalog(AnalogOperation.DriveTrainTurnY);
        double Vcy = this.driver.getAnalog(AnalogOperation.DriveTrainMoveForward);
        double Vcx = this.driver.getAnalog(AnalogOperation.DriveTrainMoveSide);

        double omega = (Math.atan2(turnX, turnY) * Helpers.RADIANS_TO_DEGREES);
        
        for(int i = 0; i < 4; i++)
        {
            double Vx = Vcx - omega * Ry[i];
            double Vy = Vcy + omega * Rx[i]; // quik mafs

            double anglePositionGoal = Math.atan2(-Vx, Vy) * Helpers.RADIANS_TO_DEGREES;
            double driveVelocityGoal = Math.sqrt(Vx * Vx + Vy * Vy);

            driveVelocityGoal = this.applyPowerLevelRange(driveVelocityGoal);

            Helpers.EnforceRange(anglePositionGoal, -180.0, 180.0);
            this.assertPowerLevelRange(driveVelocityGoal, "drive");

            driveVelocityGoal *= TuningConstants.DRIVETRAIN_DRIVE_MOTOR_1_VELOCITY_PID_KS; // only uses KS from module 1 but it shouldn't
            anglePositionGoal *= TuningConstants.DRIVETRAIN_ANGLE_MOTOR_1_POSITION_PID_KS; // matter rn cuz theyre all the same

            result.add(new Setpoint(driveVelocityGoal, anglePositionGoal));
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
}