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
    private final IAnalogInput absoluteEncoder;

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
        
        this.angleMotor1 = provider.getTalonFX(ElectronicsConstants.DRIVETRAIN_ANGLE_MOTOR_CAN_ID);
        this.angleMotor1.setInvertOutput(HardwareConstants.DRIVETRAIN_ANGLE_MOTOR_INVERT_OUTPUT);
        this.angleMotor1.setInvertSensor(HardwareConstants.DRIVETRAIN_ANGLE_MOTOR_INVERT_SENSOR);
        this.angleMotor1.setNeutralMode(MotorNeutralMode.Brake);
        this.angleMotor1.setSensorType(TalonXFeedbackDevice.IntegratedSensor);
        //this.angleMotor.setPosition((int)this.encoderAngle); //would this work?
        this.angleMotor.setPIDF(
            TuningConstants.DRIVETRAIN_ANGLE_MOTOR_POSITION_PID_KP,
            TuningConstants.DRIVETRAIN_ANGLE_MOTOR_POSITION_PID_KI,
            TuningConstants.DRIVETRAIN_ANGLE_MOTOR_POSITION_PID_KD,
            TuningConstants.DRIVETRAIN_ANGLE_MOTOR_POSITION_PID_KF,
            DriveTrainMechanism.pidSlotId);

        this.driveMotor1 = provider.getTalonFX(ElectronicsConstants.DRIVETRAIN_DRIVE_MOTOR_CAN_ID);
        this.driveMotor1.setNeutralMode(MotorNeutralMode.Brake);
        this.driveMotor1.setInvertOutput(HardwareConstants.DRIVETRAIN_DRIVE_MOTOR_INVERT_OUTPUT);
        this.driveMotor1.setInvertSensor(HardwareConstants.DRIVETRAIN_DRIVE_MOTOR_INVERT_SENSOR);
        this.driveMotor1.setSensorType(TalonXFeedbackDevice.IntegratedSensor);
        this.driveMotor1.setFeedbackFramePeriod(DriveTrainMechanism.FRAME_PERIOD_MS);
        this.driveMotor1.setPIDFFramePeriod(DriveTrainMechanism.FRAME_PERIOD_MS);
        this.driveMotor1.configureVelocityMeasurements(10, 32);
        this.driveMotor1.setPIDF(
            TuningConstants.DRIVETRAIN_DRIVE_MOTOR_VELOCITY_PID_KP,
            TuningConstants.DRIVETRAIN_DRIVE_MOTOR_VELOCITY_PID_KI,
            TuningConstants.DRIVETRAIN_DRIVE_MOTOR_VELOCITY_PID_KD,
            TuningConstants.DRIVETRAIN_DRIVE_MOTOR_VELOCITY_PID_KF,
            DriveTrainMechanism.pidSlotId);
        this.driveMotor.setVoltageCompensation(
            TuningConstants.DRIVETRAIN_VOLTAGE_COMPENSATION_ENABLED,
            TuningConstants.DRIVETRAIN_VOLTAGE_COMPENSATION);
        this.driveMotor.setSupplyCurrentLimit(
            TuningConstants.DRIVETRAIN_SUPPLY_CURRENT_LIMITING_ENABLED,
            TuningConstants.DRIVETRAIN_SUPPLY_CURRENT_MAX,
            TuningConstants.DRIVETRAIN_SUPPLY_TRIGGER_CURRENT,
            TuningConstants.DRIVETRAIN_SUPPLY_TRIGGER_DURATION);

        if (TuningConstants.DRIVETRAIN_USE_PID)
        {
            this.angleMotor.setControlMode(TalonSRXControlMode.Position);
            this.driveMotor.setControlMode(TalonSRXControlMode.Velocity);
        }
        else
        {
            this.angleMotor.setControlMode(TalonSRXControlMode.PercentOutput);
            this.driveMotor.setControlMode(TalonSRXControlMode.PercentOutput);
        }

        this.absoluteEncoder = provider.getAnalogInput(ElectronicsConstants.DRIVETRAIN_ABSOLUTE_ENCODER_ANALOG_INPUT);
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
        double a = 0.0;
        double b = 0.0;

        a1 = a - this.width/2;
        a2 = a + this.width/2;
        b1 = b - this.length/2;
        b2 = b + this.length/2;

        int[] Rx = {a1, a2, a2, a1};
        int[] Ry = {b1, b1, b2, b2};

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
            self.raw_wheel_vel.append(sqrt(Vwx*Vwx+Vwy*Vwy))*/

    }
}