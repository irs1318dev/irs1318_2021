/*
 * DriveTrainMechanism
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
public class DriveTrainMechanism implements IMechanism
{
    private static final int pidSlotId = 0;
    private static final int FRAME_PERIOD_MS = 5;

    private static final double POWERLEVEL_MIN = -1.0;
    private static final double POWERLEVEL_MAX = 1.0;

    private final double length = 10.0;
    private final double width = 10.0;

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

    private Setpoint calculateSetpoint(int module)
    {
        double a = 0.0; // center of rotation set to center of robot for now
        double b = 0.0;

        double a1 = a - this.width/2;
        double a2 = a + this.width/2;
        double b1 = b - this.length/2;
        double b2 = b + this.length/2;

        double[] Rx = {a1, a2, a2, a1}; // quik mafs
        double[] Ry = {b1, b1, b2, b2};

        double driveVelocityGoal = 0.0;
        double anglePositionGoal = 0.0;

        double turnX = this.driver.getAnalog(AnalogOperation.DriveTrainTurnX);
        double turnY = this.driver.getAnalog(AnalogOperation.DriveTrainTurnY);
        double forwardVelocity = this.driver.getAnalog(AnalogOperation.DriveTrainMoveForward);

        driveVelocityGoal = forwardVelocity;
        anglePositionGoal = (Math.atan2(turnX, turnY) * Helpers.RADIANS_TO_DEGREES);

        /*
        Vwx = Vcx - omega*Ry[i]
            Vwy = Vcy + omega*Rx[i]
            self.raw_wheel_ang.append(atan2(-Vwx,Vwy))  
            self.raw_wheel_vel.append(sqrt(Vwx*Vwx+Vwy*Vwy)) */

    }
}