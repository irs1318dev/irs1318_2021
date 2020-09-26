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

    private final double length = HardwareConstants.DRIVETRAIN_VERTICAL_WHEEL_SEPERATION_DISTANCE;
    private final double width = HardwareConstants.DRIVETRAIN_HORIZONTAL_WHEEL_SEPERATION_DISTANCE;

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


        this.absoluteEncoder1 = provider.getAnalogInput(ElectronicsConstants.DRIVETRAIN_ABSOLUTE_ENCODER_1_ANALOG_INPUT);        
        this.absoluteEncoder2 = provider.getAnalogInput(ElectronicsConstants.DRIVETRAIN_ABSOLUTE_ENCODER_2_ANALOG_INPUT);
        this.absoluteEncoder3 = provider.getAnalogInput(ElectronicsConstants.DRIVETRAIN_ABSOLUTE_ENCODER_3_ANALOG_INPUT);
        this.absoluteEncoder4 = provider.getAnalogInput(ElectronicsConstants.DRIVETRAIN_ABSOLUTE_ENCODER_4_ANALOG_INPUT);
        

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
        LoggingKey[] driveTrainLoggingDoubles = {LoggingKey.DriveTrainDriveVelocity1, LoggingKey.DriveTrainDriveError1, LoggingKey.DriveTrainAngleVelocity1, LoggingKey.DriveTrainAngleError1, LoggingKey.DriveTrainAbsoluteEncoderPosition1,
                                              LoggingKey.DriveTrainDriveVelocity2, LoggingKey.DriveTrainDriveError2, LoggingKey.DriveTrainAngleVelocity2, LoggingKey.DriveTrainAngleError2, LoggingKey.DriveTrainAbsoluteEncoderPosition2,
                                              LoggingKey.DriveTrainDriveVelocity3, LoggingKey.DriveTrainDriveError3, LoggingKey.DriveTrainAngleVelocity3, LoggingKey.DriveTrainAngleError3, LoggingKey.DriveTrainAbsoluteEncoderPosition3,
                                              LoggingKey.DriveTrainDriveVelocity4, LoggingKey.DriveTrainDriveError4, LoggingKey.DriveTrainAngleVelocity4, LoggingKey.DriveTrainAngleError4, LoggingKey.DriveTrainAbsoluteEncoderPosition4
                                            };

        LoggingKey[] driveTrainLoggingInts = {LoggingKey.DriveTrainDrivePosition1, LoggingKey.DriveTrainAnglePosition1, LoggingKey.DriveTrainDrivePosition2, LoggingKey.DriveTrainAnglePosition2,
                                                 LoggingKey.DriveTrainDrivePosition3, LoggingKey.DriveTrainAnglePosition3, LoggingKey.DriveTrainDrivePosition4, LoggingKey.DriveTrainAnglePosition4};
        
        double[] driveTrainDoubles = {this.driveVelocity1, this.driveError1, this.angleVelocity1, this.angleError1, this.encoderAngle1,
                                      this.driveVelocity2, this.driveError2, this.angleVelocity2, this.angleError2, this.encoderAngle2,
                                      this.driveVelocity3, this.driveError3, this.angleVelocity3, this.angleError3, this.encoderAngle3,
                                      this.driveVelocity4, this.driveError4, this.angleVelocity4, this.angleError4, this.encoderAngle4
                                    };

        int[] driveTrainInts = {this.drivePosition1, this.anglePosition1, this.drivePosition2, this.anglePosition2,
                                this.drivePosition3, this.anglePosition3, this.drivePosition4, this.anglePosition4};

        for(int i = 0; i<=19; i++);{
            this.logger.logNumber(driveTrainLoggingDoubles[i], driveTrainDoubles[i]);
        }
        for(int v = 0; v<=7; v++);{
            this.logger.logNumber(driveTrainLoggingInts[v], driveTrainInts[v]);
        }
        /*
        this.logger.logNumber(LoggingKey.DriveTrainDriveVelocity1, this.driveVelocity1);
        this.logger.logNumber(LoggingKey.DriveTrainDriveError1, this.driveError1);
        this.logger.logNumber(LoggingKey.DriveTrainDrivePosition1, this.drivePosition1); //
        this.logger.logNumber(LoggingKey.DriveTrainAngleVelocity1, this.angleVelocity1);
        this.logger.logNumber(LoggingKey.DriveTrainAngleError1, this.angleError1);
        this.logger.logNumber(LoggingKey.DriveTrainAnglePosition1, this.anglePosition1); //
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
        */
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
                angleMotors[i].getPosition(), 
                -180.0, // wasn't too sure what these were supposed to be
                180.0);
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
        ITalonFX[] angleMotors = {this.angleMotor1, this.angleMotor2, this.angleMotor3, this.angleMotor4};
        ITalonFX[] driveMotors = {this.driveMotor1, this.driveMotor2, this.driveMotor3, this.driveMotor4};
        double[] doubleFields = {this.driveVelocity1, this.driveVelocity2, this.driveVelocity3, this.driveVelocity4, this.driveError1, this.driveError2, this.driveError3, this.driveError4, this.angleVelocity1, this.angleVelocity2, this.angleVelocity3, this.angleVelocity4, this.angleError1, this.angleError2, this.angleError3, this.angleError4};
        int[] intFields = {this.drivePosition1, this.drivePosition2, this.drivePosition3, this.drivePosition4, this.anglePosition1, this.anglePosition2, this.anglePosition3, this.anglePosition4};

        for(int i = 0; i < 4; i++)
        {
            driveMotors[i].stop();
            angleMotors[i].stop();
        }

        for (int i = 0; i < intFields.length; i++) 
        { 
            intFields[i] = 0; 
        } 
        for (int i = 0; i < doubleFields.length; i++) 
        { 
            doubleFields[i] = 0.0; 
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
        
        for(int i = 0; i < 4; i++)
        {
            double Vx = Vcx - omega * Ry[i];
            double Vy = Vcy + omega * Rx[i]; // quik mafs

            double anglePositionGoal = Math.atan2(-Vx, Vy);
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