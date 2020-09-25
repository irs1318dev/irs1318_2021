/*
 * DriveTrainMechanism
 * 
 * authors: Will, Vanshika, Arushi
 * 
 * Started idk sometime in september
*/

package frc.robot.mechanisms;

import java.util.BitSet;

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

<<<<<<< HEAD
    private final double length = HardwareConstants.DRIVETRAIN_VERTICAL_WHEEL_SEPARATION_DISTANCE;
    private final double width = HardwareConstants.DRIVETRAIN_HORIZONTAL_WHEEL_SEPARATION_DISTANCE;
=======
    private final double length = HardwareConstants.DRIVETRAIN_VERTICAL_WHEEL_SEPERATION_DISTANCE;
    private final double width = HardwareConstants.DRIVETRAIN_HORIZONTAL_WHEEL_SEPERATION_DISTANCE;
>>>>>>> 431d6cefc2039bca0f385b03d8e3885737bf8b48

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

    private double driveVelocity1;
    private double driveError1;
    private int drivePosition1;
    private double angleVelocity1;
    private double angleError1;
    private int anglePosition1;

    private double encoderVoltage1;
    private double encoderAngle1;

    private double driveVelocity2;
    private double driveError2;
    private int drivePosition2;
    private double angleVelocity2;
    private double angleError2;
    private int anglePosition2;

    private double encoderVoltage2;
    private double encoderAngle2;

    private double driveVelocity3;
    private double driveError3;
    private int drivePosition3;
    private double angleVelocity3;
    private double angleError3;
    private int anglePosition3;

    private double encoderVoltage3;
    private double encoderAngle3;

    private double driveVelocity4;
    private double driveError4;
    private int drivePosition4;
    private double angleVelocity4;
    private double angleError4;
    private int anglePosition4;

    private double encoderVoltage4;
    private double encoderAngle4;


    @Inject
    public DriveTrainMechanism(
        LoggingManager logger,
        IRobotProvider provider)
    {
        this.logger = logger;
        
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

        this.driveMotor1 = provider.getTalonFX(ElectronicsConstants.DRIVETRAIN_DRIVE_MOTOR_1_CAN_ID);
        this.driveMotor1.setInvertOutput(HardwareConstants.DRIVETRAIN_DRIVE_MOTOR_1_INVERT_OUTPUT);
        this.driveMotor1.setInvertSensor(HardwareConstants.DRIVETRAIN_DRIVE_MOTOR_1_INVERT_SENSOR);
        this.driveMotor1.setNeutralMode(MotorNeutralMode.Brake); //
        this.driveMotor1.setSensorType(TalonXFeedbackDevice.IntegratedSensor); //
        this.driveMotor1.setFeedbackFramePeriod(DriveTrainMechanism.FRAME_PERIOD_MS); //
        this.driveMotor1.setPIDFFramePeriod(DriveTrainMechanism.FRAME_PERIOD_MS); //
        this.driveMotor1.configureVelocityMeasurements(10, 32); //
        this.driveMotor1.setPIDF(
            TuningConstants.DRIVETRAIN_DRIVE_MOTOR_1_VELOCITY_PID_KP,
            TuningConstants.DRIVETRAIN_DRIVE_MOTOR_1_VELOCITY_PID_KI,
            TuningConstants.DRIVETRAIN_DRIVE_MOTOR_1_VELOCITY_PID_KD,
            TuningConstants.DRIVETRAIN_DRIVE_MOTOR_1_VELOCITY_PID_KF,
            DriveTrainMechanism.pidSlotId);
        this.driveMotor1.setVoltageCompensation(
            TuningConstants.DRIVETRAIN_VOLTAGE_COMPENSATION_ENABLED,
            TuningConstants.DRIVETRAIN_VOLTAGE_COMPENSATION);
        this.driveMotor1.setSupplyCurrentLimit(
            TuningConstants.DRIVETRAIN_SUPPLY_CURRENT_LIMITING_ENABLED,
            TuningConstants.DRIVETRAIN_SUPPLY_CURRENT_MAX,
            TuningConstants.DRIVETRAIN_SUPPLY_TRIGGER_CURRENT,
            TuningConstants.DRIVETRAIN_SUPPLY_TRIGGER_DURATION);

        if (TuningConstants.DRIVETRAIN_USE_PID)
        {
            this.angleMotor1.setControlMode(TalonSRXControlMode.Position);
            this.driveMotor1.setControlMode(TalonSRXControlMode.Velocity);
            this.angleMotor2.setControlMode(TalonSRXControlMode.Position);
            this.driveMotor2.setControlMode(TalonSRXControlMode.Velocity);
            this.angleMotor3.setControlMode(TalonSRXControlMode.Position);
            this.driveMotor3.setControlMode(TalonSRXControlMode.Velocity);
            this.angleMotor4.setControlMode(TalonSRXControlMode.Position);
            this.driveMotor4.setControlMode(TalonSRXControlMode.Velocity);
        }
        else
        {
            this.angleMotor1.setControlMode(TalonSRXControlMode.PercentOutput);
            this.driveMotor1.setControlMode(TalonSRXControlMode.PercentOutput);
            this.angleMotor2.setControlMode(TalonSRXControlMode.PercentOutput);
            this.driveMotor2.setControlMode(TalonSRXControlMode.PercentOutput);
            this.angleMotor3.setControlMode(TalonSRXControlMode.PercentOutput);
            this.driveMotor3.setControlMode(TalonSRXControlMode.PercentOutput);
            this.angleMotor4.setControlMode(TalonSRXControlMode.PercentOutput);
            this.driveMotor4.setControlMode(TalonSRXControlMode.PercentOutput);
        }

        this.absoluteEncoder1 = provider.getAnalogInput(ElectronicsConstants.DRIVETRAIN_ABSOLUTE_ENCODER_1_ANALOG_INPUT);



        //MODULE 2
        this.angleMotor2 = provider.getTalonFX(ElectronicsConstants.DRIVETRAIN_ANGLE_MOTOR_2_CAN_ID);
        this.angleMotor2.setInvertOutput(HardwareConstants.DRIVETRAIN_ANGLE_MOTOR_2_INVERT_OUTPUT);
        this.angleMotor2.setInvertSensor(HardwareConstants.DRIVETRAIN_ANGLE_MOTOR_2_INVERT_SENSOR);
        this.angleMotor2.setNeutralMode(MotorNeutralMode.Brake);
        this.angleMotor2.setSensorType(TalonXFeedbackDevice.IntegratedSensor);
        //this.angleMotor.setPosition((int)this.encoderAngle); //would this work?
        this.angleMotor2.setPIDF(
            TuningConstants.DRIVETRAIN_ANGLE_MOTOR_2_POSITION_PID_KP,
            TuningConstants.DRIVETRAIN_ANGLE_MOTOR_2_POSITION_PID_KI,
            TuningConstants.DRIVETRAIN_ANGLE_MOTOR_2_POSITION_PID_KD,
            TuningConstants.DRIVETRAIN_ANGLE_MOTOR_2_POSITION_PID_KF,
            DriveTrainMechanism.pidSlotId);

        this.driveMotor2 = provider.getTalonFX(ElectronicsConstants.DRIVETRAIN_DRIVE_MOTOR_2_CAN_ID);
        this.driveMotor2.setInvertOutput(HardwareConstants.DRIVETRAIN_DRIVE_MOTOR_2_INVERT_OUTPUT);
        this.driveMotor2.setInvertSensor(HardwareConstants.DRIVETRAIN_DRIVE_MOTOR_2_INVERT_SENSOR);
        this.driveMotor2.setNeutralMode(MotorNeutralMode.Brake); //
        this.driveMotor2.setSensorType(TalonXFeedbackDevice.IntegratedSensor); //
        this.driveMotor2.setFeedbackFramePeriod(DriveTrainMechanism.FRAME_PERIOD_MS); //
        this.driveMotor2.setPIDFFramePeriod(DriveTrainMechanism.FRAME_PERIOD_MS); //
        this.driveMotor2.configureVelocityMeasurements(10, 32); //
        this.driveMotor2.setPIDF(
            TuningConstants.DRIVETRAIN_DRIVE_MOTOR_2_VELOCITY_PID_KP,
            TuningConstants.DRIVETRAIN_DRIVE_MOTOR_2_VELOCITY_PID_KI,
            TuningConstants.DRIVETRAIN_DRIVE_MOTOR_2_VELOCITY_PID_KD,
            TuningConstants.DRIVETRAIN_DRIVE_MOTOR_2_VELOCITY_PID_KF,
            DriveTrainMechanism.pidSlotId);
        this.driveMotor2.setVoltageCompensation(
            TuningConstants.DRIVETRAIN_VOLTAGE_COMPENSATION_ENABLED,
            TuningConstants.DRIVETRAIN_VOLTAGE_COMPENSATION);
        this.driveMotor2.setSupplyCurrentLimit(
            TuningConstants.DRIVETRAIN_SUPPLY_CURRENT_LIMITING_ENABLED,
            TuningConstants.DRIVETRAIN_SUPPLY_CURRENT_MAX,
            TuningConstants.DRIVETRAIN_SUPPLY_TRIGGER_CURRENT,
            TuningConstants.DRIVETRAIN_SUPPLY_TRIGGER_DURATION);

        this.absoluteEncoder2 = provider.getAnalogInput(ElectronicsConstants.DRIVETRAIN_ABSOLUTE_ENCODER_2_ANALOG_INPUT);



        //MODULE 3
        this.angleMotor3 = provider.getTalonFX(ElectronicsConstants.DRIVETRAIN_ANGLE_MOTOR_2_CAN_ID);
        this.angleMotor3.setInvertOutput(HardwareConstants.DRIVETRAIN_ANGLE_MOTOR_2_INVERT_OUTPUT);
        this.angleMotor3.setInvertSensor(HardwareConstants.DRIVETRAIN_ANGLE_MOTOR_2_INVERT_SENSOR);
        this.angleMotor3.setNeutralMode(MotorNeutralMode.Brake);
        this.angleMotor3.setSensorType(TalonXFeedbackDevice.IntegratedSensor);
        //this.angleMotor.setPosition((int)this.encoderAngle); //would this work?
        this.angleMotor3.setPIDF(
            TuningConstants.DRIVETRAIN_ANGLE_MOTOR_3_POSITION_PID_KP,
            TuningConstants.DRIVETRAIN_ANGLE_MOTOR_3_POSITION_PID_KI,
            TuningConstants.DRIVETRAIN_ANGLE_MOTOR_3_POSITION_PID_KD,
            TuningConstants.DRIVETRAIN_ANGLE_MOTOR_3_POSITION_PID_KF,
            DriveTrainMechanism.pidSlotId);

        this.driveMotor3 = provider.getTalonFX(ElectronicsConstants.DRIVETRAIN_DRIVE_MOTOR_3_CAN_ID);
        this.driveMotor3.setInvertOutput(HardwareConstants.DRIVETRAIN_DRIVE_MOTOR_3_INVERT_OUTPUT);
        this.driveMotor3.setInvertSensor(HardwareConstants.DRIVETRAIN_DRIVE_MOTOR_3_INVERT_SENSOR);
        this.driveMotor3.setNeutralMode(MotorNeutralMode.Brake); //
        this.driveMotor3.setSensorType(TalonXFeedbackDevice.IntegratedSensor); //
        this.driveMotor3.setFeedbackFramePeriod(DriveTrainMechanism.FRAME_PERIOD_MS); //
        this.driveMotor3.setPIDFFramePeriod(DriveTrainMechanism.FRAME_PERIOD_MS); //
        this.driveMotor3.configureVelocityMeasurements(10, 32); //
        this.driveMotor3.setPIDF(
            TuningConstants.DRIVETRAIN_DRIVE_MOTOR_3_VELOCITY_PID_KP,
            TuningConstants.DRIVETRAIN_DRIVE_MOTOR_3_VELOCITY_PID_KI,
            TuningConstants.DRIVETRAIN_DRIVE_MOTOR_3_VELOCITY_PID_KD,
            TuningConstants.DRIVETRAIN_DRIVE_MOTOR_3_VELOCITY_PID_KF,
            DriveTrainMechanism.pidSlotId);
        this.driveMotor3.setVoltageCompensation(
            TuningConstants.DRIVETRAIN_VOLTAGE_COMPENSATION_ENABLED,
            TuningConstants.DRIVETRAIN_VOLTAGE_COMPENSATION);
        this.driveMotor3.setSupplyCurrentLimit(
            TuningConstants.DRIVETRAIN_SUPPLY_CURRENT_LIMITING_ENABLED,
            TuningConstants.DRIVETRAIN_SUPPLY_CURRENT_MAX,
            TuningConstants.DRIVETRAIN_SUPPLY_TRIGGER_CURRENT,
            TuningConstants.DRIVETRAIN_SUPPLY_TRIGGER_DURATION);

        this.absoluteEncoder3 = provider.getAnalogInput(ElectronicsConstants.DRIVETRAIN_ABSOLUTE_ENCODER_3_ANALOG_INPUT);


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
        this.driveMotor4.setNeutralMode(MotorNeutralMode.Brake); //
        this.driveMotor4.setSensorType(TalonXFeedbackDevice.IntegratedSensor); //
        this.driveMotor4.setFeedbackFramePeriod(DriveTrainMechanism.FRAME_PERIOD_MS); //
        this.driveMotor4.setPIDFFramePeriod(DriveTrainMechanism.FRAME_PERIOD_MS); //
        this.driveMotor4.configureVelocityMeasurements(10, 32); //WHAT IS THIS?
        this.driveMotor4.setPIDF(
            TuningConstants.DRIVETRAIN_DRIVE_MOTOR_4_VELOCITY_PID_KP,
            TuningConstants.DRIVETRAIN_DRIVE_MOTOR_4_VELOCITY_PID_KI,
            TuningConstants.DRIVETRAIN_DRIVE_MOTOR_4_VELOCITY_PID_KD,
            TuningConstants.DRIVETRAIN_DRIVE_MOTOR_4_VELOCITY_PID_KF,
            DriveTrainMechanism.pidSlotId);
        this.driveMotor4.setVoltageCompensation(
            TuningConstants.DRIVETRAIN_VOLTAGE_COMPENSATION_ENABLED,
            TuningConstants.DRIVETRAIN_VOLTAGE_COMPENSATION);
        this.driveMotor4.setSupplyCurrentLimit(
            TuningConstants.DRIVETRAIN_SUPPLY_CURRENT_LIMITING_ENABLED,
            TuningConstants.DRIVETRAIN_SUPPLY_CURRENT_MAX,
            TuningConstants.DRIVETRAIN_SUPPLY_TRIGGER_CURRENT,
            TuningConstants.DRIVETRAIN_SUPPLY_TRIGGER_DURATION);

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
        this.encoderVoltage1 = this.absoluteEncoder1.getVoltage();
        this.encoderVoltage2 = this.absoluteEncoder2.getVoltage();
        this.encoderVoltage3 = this.absoluteEncoder3.getVoltage();
        this.encoderVoltage4 = this.absoluteEncoder4.getVoltage();
        this.encoderAngle1 = this.encoderVoltage1 * HardwareConstants.DRIVETRAIN_ENCODER_DEGREES_PER_VOLT;
        this.encoderAngle2 = this.encoderVoltage2 * HardwareConstants.DRIVETRAIN_ENCODER_DEGREES_PER_VOLT;
        this.encoderAngle3 = this.encoderVoltage3 * HardwareConstants.DRIVETRAIN_ENCODER_DEGREES_PER_VOLT;
        this.encoderAngle4 = this.encoderVoltage4 * HardwareConstants.DRIVETRAIN_ENCODER_DEGREES_PER_VOLT;
    
        //NUMBER 1
        this.driveVelocity1 = this.driveMotor1.getVelocity();
        this.angleVelocity1 = this.angleMotor1.getVelocity();

        this.drivePosition1 = this.driveMotor1.getPosition();
        this.anglePosition1 = this.angleMotor1.getPosition();

        this.driveError1 = this.driveMotor1.getError();
        this.angleError1 = this.angleMotor1.getError();
        
        //NUMBER 2
        this.driveVelocity2 = this.driveMotor2.getVelocity();
        this.angleVelocity2 = this.angleMotor2.getVelocity();

        this.drivePosition2 = this.driveMotor2.getPosition();
        this.anglePosition2 = this.angleMotor2.getPosition();

        this.driveError2 = this.driveMotor2.getError();
        this.angleError2 = this.angleMotor2.getError();

        //NUMBER 3
        this.driveVelocity3 = this.driveMotor3.getVelocity();
        this.angleVelocity3 = this.angleMotor3.getVelocity();

        this.drivePosition3 = this.driveMotor3.getPosition();
        this.anglePosition3 = this.angleMotor3.getPosition();

        this.driveError3 = this.driveMotor3.getError();
        this.angleError3 = this.angleMotor3.getError();

        //NUMBER 4
        this.driveVelocity4 = this.driveMotor4.getVelocity();
        this.angleVelocity4 = this.angleMotor4.getVelocity();

        this.drivePosition4 = this.driveMotor4.getPosition();
        this.anglePosition4 = this.angleMotor4.getPosition();

        this.driveError4 = this.driveMotor4.getError();
        this.angleError4 = this.angleMotor4.getError();
        

        //LOGGING KEYS
        this.logger.logNumber(LoggingKey.DriveTrainDriveVelocity1, this.driveVelocity1);
        this.logger.logNumber(LoggingKey.DriveTrainDriveError1, this.driveError1);
        this.logger.logNumber(LoggingKey.DriveTrainDrivePosition1, this.drivePosition1);
        this.logger.logNumber(LoggingKey.DriveTrainAngleVelocity1, this.angleVelocity1);
        this.logger.logNumber(LoggingKey.DriveTrainAngleError1, this.angleError1);
        this.logger.logNumber(LoggingKey.DriveTrainAnglePosition1, this.anglePosition1);
        this.logger.logNumber(LoggingKey.DriveTrainAbsoluteEncoderPosition1, this.encoderAngle1);

        this.logger.logNumber(LoggingKey.DriveTrainDriveVelocity2, this.driveVelocity2);
        this.logger.logNumber(LoggingKey.DriveTrainDriveError2, this.driveError2);
        this.logger.logNumber(LoggingKey.DriveTrainDrivePosition2, this.drivePosition2);
        this.logger.logNumber(LoggingKey.DriveTrainAngleVelocity2, this.angleVelocity2);
        this.logger.logNumber(LoggingKey.DriveTrainAngleError2, this.angleError2);
        this.logger.logNumber(LoggingKey.DriveTrainAnglePosition2, this.anglePosition2);
        this.logger.logNumber(LoggingKey.DriveTrainAbsoluteEncoderPosition2, this.encoderAngle2);

        this.logger.logNumber(LoggingKey.DriveTrainDriveVelocity3, this.driveVelocity3);
        this.logger.logNumber(LoggingKey.DriveTrainDriveError3, this.driveError3);
        this.logger.logNumber(LoggingKey.DriveTrainDrivePosition3, this.drivePosition3);
        this.logger.logNumber(LoggingKey.DriveTrainAngleVelocity3, this.angleVelocity3);
        this.logger.logNumber(LoggingKey.DriveTrainAngleError3, this.angleError3);
        this.logger.logNumber(LoggingKey.DriveTrainAnglePosition3, this.anglePosition3);
        this.logger.logNumber(LoggingKey.DriveTrainAbsoluteEncoderPosition3, this.encoderAngle3);

        this.logger.logNumber(LoggingKey.DriveTrainDriveVelocity4, this.driveVelocity4);
        this.logger.logNumber(LoggingKey.DriveTrainDriveError4, this.driveError4);
        this.logger.logNumber(LoggingKey.DriveTrainDrivePosition4, this.drivePosition4);
        this.logger.logNumber(LoggingKey.DriveTrainAngleVelocity4, this.angleVelocity4);
        this.logger.logNumber(LoggingKey.DriveTrainAngleError4, this.angleError4);
        this.logger.logNumber(LoggingKey.DriveTrainAnglePosition4, this.anglePosition4);
        this.logger.logNumber(LoggingKey.DriveTrainAbsoluteEncoderPosition4, this.encoderAngle4);
    }

    public void update()
    {
        Setpoint setpoint1 = this.calculateSetpoint(1);
        Setpoint setpoint2 = this.calculateSetpoint(2);
        Setpoint setpoint3 = this.calculateSetpoint(3);
        Setpoint setpoint4 = this.calculateSetpoint(4);

        double driveSetpoint1 = setpoint1.getDrive();
        double angleSetpoint1 = setpoint1.getAngle();

        double driveSetpoint2 = setpoint2.getDrive();
        double angleSetpoint2 = setpoint2.getAngle();

        double driveSetpoint3 = setpoint3.getDrive();
        double angleSetpoint3 = setpoint3.getAngle();

        double driveSetpoint4 = setpoint4.getDrive();
        double angleSetpoint4 = setpoint4.getAngle();

        double bestAngle1 = getClosestAngleInRange(angleSetpoint1, this.angleMotor1.getPosition(), -Math.pi, Math.pi);
        double bestAngle2 = getClosestAngleInRange(angleSetpoint2, this.angleMotor2.getPosition(), -Math.pi, Math.pi);
        double bestAngle3 = getClosestAngleInRange(angleSetpoint3, this.angleMotor3.getPosition(), -Math.pi, Math.pi);
        double bestAngle4 = getClosestAngleInRange(angleSetpoint4, this.angleMotor4.getPosition(), -Math.pi, Math.pi);
        this.logger.logNumber(LoggingKey.DriveTrainDriveVelocityGoal, driveSetpoint);
        this.logger.logNumber(LoggingKey.DriveTrainAnglePositionGoal, angleSetpoint);

        // apply the setpoints to the motors
        this.driveMotor1.set(driveSetpoint1);
        this.angleMotor1.set(bestAngle1);

        this.driveMotor2.set(driveSetpoint2);
        this.angleMotor2.set(bestAngle2);

        this.driveMotor3.set(driveSetpoint3);
        this.angleMotor3.set(bestAngle3);

        this.driveMotor4.set(driveSetpoint4);
        this.angleMotor4.set(bestAngle4);


        if (this.driver.getDigital(DigitalOperation.DriveTrainReset))
        {
            this.angleMotor1.reset();
            this.driveMotor1.reset();
            this.angleMotor2.reset();
            this.driveMotor2.reset();
            this.angleMotor3.reset();
            this.driveMotor3.reset();
            this.angleMotor4.reset();
            this.driveMotor4.reset();
        }
    }

    private double getClosestAngleInRange(double desiredAngle, double currentAngle, double minRangeValue, double maxRangeValue)
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
        this.driveMotor.stop();
        this.angleMotor.stop();

        this.driveVelocity = 0.0;
        this.driveError = 0.0;
        this.drivePosition = 0;
        this.angleVelocity = 0.0;
        this.angleError = 0.0;
        this.anglePosition = 0;
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


    private Setpoint calculateSetpoint(int module)
    {
        double a = 0.0; // center of rotation set to center of robot for now
        double b = 0.0;

        double a1 = a - this.width/2;
        double a2 = a + this.width/2;
        double b1 = b - this.length/2;
        double b2 = b + this.length/2;

        double[] Rx = {a1, a2, a2, a1}; 
        double[] Ry = {b1, b1, b2, b2};

        double turnX = this.driver.getAnalog(AnalogOperation.DriveTrainTurnX);
        double turnY = this.driver.getAnalog(AnalogOperation.DriveTrainTurnY);
        double Vcy = this.driver.getAnalog(AnalogOperation.DriveTrainMoveForward);
        double Vcx = this.driver.getAnalog(AnalogOperation.DriveTrainMoveSide);

        double omega = (Math.atan2(turnX, turnY) * Helpers.RADIANS_TO_DEGREES);

        double Vx = Vcx - omega * Ry[module-1];
        double Vy = Vcy + omega * Rx[module-1]; // quik mafs

        double anglePositionGoal = Math.atan2(-Vx, Vy);
        double driveVelocityGoal = Math.sqrt(Vx * Vx + Vy * Vy);

        driveVelocityGoal = this.applyPowerLevelRange(driveVelocityGoal);

        Helpers.EnforceRange(anglePositionGoal, -180.0, 180.0);
        this.assertPowerLevelRange(driveVelocityGoal, "drive");

        driveVelocityGoal *= TuningConstants.DRIVETRAIN_DRIVE_MOTOR_1_VELOCITY_PID_KS;
        anglePositionGoal *= TuningConstants.DRIVETRAIN_ANGLE_MOTOR_1_POSITION_PID_KS;

        return new Setpoint(driveVelocityGoal, anglePositionGoal);
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