/**
 * DriveTrainMechanism
 * 
 * @author Will, Vanshika, Arushi
 * 
 * Started idk sometime in september
 * 
 * dO yOu ReMeMbEr 
 * tHe 21sT nIgHt oF sEpTeMbEr
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

    private final ILogger logger;

    private Driver driver;

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

    private ITalonFX[] angleMotors;
    private ITalonFX[] driveMotors;
    private double[] encoderVoltages;
    private double[] encoderAngles;
    private double[] driveVelocities = {this.driveVelocity1, this.driveVelocity2, this.driveVelocity3, this.driveVelocity4};
    private int[] drivePositions = {this.drivePosition1, this.drivePosition2, this.drivePosition3, this.drivePosition4};
    private double[] driveErrors = {this.driveError1, this.driveError2, this.driveError3, this.driveError4};
    private double[] angleVelocities = {this.angleVelocity1, this.angleVelocity2, this.angleVelocity3, this.angleVelocity4};
    private int[] anglePositions = {this.anglePosition1, this.anglePosition2, this.anglePosition3, this.anglePosition4};;
    private double[] angleErrors = {this.angleError1, this.angleError2, this.angleError3, this.angleError4};
    private IAnalogInput[] absoluteEncoders;

    private final LoggingKey[] encoderAnglesLK = {LoggingKey.DriveTrainAbsoluteEncoderPosition1, LoggingKey.DriveTrainAbsoluteEncoderPosition2, LoggingKey.DriveTrainAbsoluteEncoderPosition3, LoggingKey.DriveTrainAbsoluteEncoderPosition4};
    private final LoggingKey[] driveVelocitiesLK = {LoggingKey.DriveTrainDriveVelocity1, LoggingKey.DriveTrainDriveVelocity2, LoggingKey.DriveTrainDriveVelocity3, LoggingKey.DriveTrainDriveVelocity4};
    private final LoggingKey[] drivePositionsLK = {LoggingKey.DriveTrainDrivePosition1, LoggingKey.DriveTrainDrivePosition2, LoggingKey.DriveTrainDrivePosition3, LoggingKey.DriveTrainDrivePosition4};
    private final LoggingKey[] driveErrorsLK = {LoggingKey.DriveTrainDriveError1, LoggingKey.DriveTrainDriveError2, LoggingKey.DriveTrainDriveError3, LoggingKey.DriveTrainDriveError4};
    private final LoggingKey[] angleVelocitiesLK = {LoggingKey.DriveTrainAngleVelocity1, LoggingKey.DriveTrainAngleVelocity2, LoggingKey.DriveTrainAngleVelocity3, LoggingKey.DriveTrainAngleVelocity4,};
    private final LoggingKey[] anglePositionsLK = {LoggingKey.DriveTrainAnglePosition1, LoggingKey.DriveTrainAnglePosition2, LoggingKey.DriveTrainAnglePosition3, LoggingKey.DriveTrainAnglePosition4};;
    private final LoggingKey[] angleErrorsLK = {LoggingKey.DriveTrainAngleError1, LoggingKey.DriveTrainAngleError2, LoggingKey.DriveTrainAngleError3, LoggingKey.DriveTrainAngleError4,};
    private final LoggingKey[] driveVelocityGoalsLK = {LoggingKey.DriveTrainDriveVelocityGoal1, LoggingKey.DriveTrainDriveVelocityGoal2, LoggingKey.DriveTrainDriveVelocityGoal3, LoggingKey.DriveTrainDriveVelocityGoal4};
    private final LoggingKey[] anglePositionGoalsLK = {LoggingKey.DriveTrainAnglePositionGoal1, LoggingKey.DriveTrainAnglePositionGoal2, LoggingKey.DriveTrainAnglePositionGoal3, LoggingKey.DriveTrainAnglePositionGoal4};
    
    @Inject
    public DriveTrainMechanism(
        LoggingManager logger,
        IRobotProvider provider)
    {
        this.logger = logger;

        this.encoderAngles = new double[4];
        this.encoderVoltages = new double[4]; 
        this.angleMotors = new ITalonFX[4];
        this.driveMotors = new ITalonFX[4];


        for(int i = 0; i <=3; i++)
        {

            this.driveMotors[i] = provider.getTalonFX(ElectronicsConstants.DRIVE_CANS[i]);
            this.driveMotors[i].setInvertOutput(HardwareConstants.driveInvertOutput[i]);
            this.driveMotors[i].setInvertSensor(HardwareConstants.driveInvertSensors[i]);
            this.driveMotors[i].setNeutralMode(MotorNeutralMode.Brake); 
            this.driveMotors[i].setSensorType(TalonXFeedbackDevice.IntegratedSensor); 
            this.driveMotors[i].setFeedbackFramePeriod(DriveTrainMechanism.FRAME_PERIOD_MS); 
            this.driveMotors[i].setPIDFFramePeriod(DriveTrainMechanism.FRAME_PERIOD_MS); 
            this.driveMotors[i].setPIDF(
                TuningConstants.driveVelocityKPs[i], 
                TuningConstants.driveVelocityKIs[i], 
                TuningConstants.driveVelocityKDs[i], 
                TuningConstants.driveVelocityKFs[i], 
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

            this.angleMotors[i] = provider.getTalonFX(ElectronicsConstants.ANGLE_CANS[i]);
            this.angleMotors[i].setInvertOutput(HardwareConstants.angleInvertOutput[i]);
            this.angleMotors[i].setInvertSensor(HardwareConstants.angleInvertSensors[i]);
            this.angleMotors[i].setNeutralMode(MotorNeutralMode.Brake);
            this.angleMotors[i].setSensorType(TalonXFeedbackDevice.IntegratedSensor);
            this.angleMotors[i].setPIDF(
                TuningConstants.anglePositionKPs[i], 
                TuningConstants.anglePositionKIs[i], 
                TuningConstants.anglePositionKDs[i], 
                TuningConstants.anglePositionKFs[i],
                DriveTrainMechanism.pidSlotId);
            this.angleMotors[i].setControlMode(TalonSRXControlMode.Position);

            this.absoluteEncoders[i] = provider.getAnalogInput(ElectronicsConstants.DRIVETRAIN_ABSOLUTE_ENCODER_ANALOG_INPUTS[i]);

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
        
        for(int i = 0; i <=3; i++)
        {
            this.encoderVoltages[i] = this.absoluteEncoders[i].getVoltage();
            this.encoderAngles[i] = this.encoderVoltages[i] * HardwareConstants.DRIVETRAIN_ENCODER_DEGREES_PER_VOLT;
            this.driveVelocities[i] = this.driveMotors[i].getVelocity();
            this.drivePositions[i] = this.driveMotors[i].getPosition();
            this.driveErrors[i] = this.driveMotors[i].getError();
            this.angleVelocities[i] = this.angleMotors[i].getVelocity();
            this.anglePositions[i] = this.angleMotors[i].getPosition();
            this.angleErrors[i] = this.angleMotors[i].getError();
            this.logger.logNumber(this.driveVelocitiesLK[i], this.driveVelocities[i]);
            this.logger.logNumber(this.angleVelocitiesLK[i], this.angleVelocities[i]);
            this.logger.logNumber(this.driveErrorsLK[i], this.driveErrors[i]);
            this.logger.logNumber(this.angleErrorsLK[i], this.angleErrors[i]);
            this.logger.logNumber(this.drivePositionsLK[i], this.drivePositions[i]);
            this.logger.logNumber(this.anglePositionsLK[i], this.anglePositions[i]);
            this.logger.logNumber(this.encoderAnglesLK[i], this.encoderAngles[i]);
        }
    }

    public void update()
    {
        Setpoint[] setpoint = this.calculateSetpoint();

        for(int i = 0; i < 4; i++)
        {
            Setpoint current = setpoint[i];

            this.logger.logNumber(this.anglePositionGoalsLK[i], current.getAngle());
            this.logger.logNumber(this.driveVelocityGoalsLK[i], current.getDrive());

            this.driveMotors[i].set(current.getDrive());
            this.angleMotors[i].set(current.getAngle());
        }

        if (this.driver.getDigital(DigitalOperation.DriveTrainReset))
        {
            for(int i = 0; i < 4; i++)
            {
                this.driveMotors[i].reset();
                this.angleMotors[i].reset();
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


         /** Initializes a new Setpoint
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


    private Setpoint[] calculateSetpoint()
    {
        double a = 0.0; // center of rotation set to center of robot for now
        double b = 0.0;

        Setpoint[] result = new Setpoint[4];
        
        double a1 = a - HardwareConstants.DRIVETRAIN_HORIZONTAL_WHEEL_SEPERATION_DISTANCE/2;
        double a2 = a + HardwareConstants.DRIVETRAIN_HORIZONTAL_WHEEL_SEPERATION_DISTANCE/2;
        double b1 = b - HardwareConstants.DRIVETRAIN_VERTICAL_WHEEL_SEPERATION_DISTANCE/2;
        double b2 = b + HardwareConstants.DRIVETRAIN_VERTICAL_WHEEL_SEPERATION_DISTANCE/2;

        double[] Rx = {a1, a2, a2, a1}; 
        double[] Ry = {b1, b1, b2, b2};

        double turnX = this.driver.getAnalog(AnalogOperation.DriveTrainTurnX);
        //double turnY = this.driver.getAnalog(AnalogOperation.DriveTrainTurnY); // only for field oriented control, unused
        double Vcy = this.driver.getAnalog(AnalogOperation.DriveTrainMoveForward);
        double Vcx = this.driver.getAnalog(AnalogOperation.DriveTrainMoveSide);

        //double omega = (Math.atan2(turnX, turnY) * Helpers.RADIANS_TO_DEGREES);
        double omega = turnX * TuningConstants.OMEGA_ANGLE_VELOCITY; 

        for(int i = 0; i < 4; i++)
        {
            double Vx = Vcx - omega * Ry[i];
            double Vy = Vcy + omega * Rx[i]; // quik mafs

            double anglePositionGoal = Math.atan2(-Vx, Vy) * Helpers.RADIANS_TO_DEGREES;
            double driveVelocityGoal = Math.sqrt(Vx * Vx + Vy * Vy);
            
            anglePositionGoal = getClosestAngleInRange(
                anglePositionGoal, 
                this.anglePositions[i]/TuningConstants.DRIVETRAIN_ANGLE_MOTOR_POSITION_PID_KS[i]);
            driveVelocityGoal = this.applyPowerLevelRange(driveVelocityGoal);

            this.assertPowerLevelRange(driveVelocityGoal, "drive");

            driveVelocityGoal *= TuningConstants.DRIVETRAIN_DRIVE_MOTOR_1_VELOCITY_PID_KS; 
            anglePositionGoal *= TuningConstants.DRIVETRAIN_ANGLE_MOTOR_1_POSITION_PID_KS; 

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
}