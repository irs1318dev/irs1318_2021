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
import frc.robot.driver.common.IDriver;

import com.google.inject.Inject;
import com.google.inject.Singleton;

@Singleton
public class DriveTrainMechanism implements IMechanism
{
    private static final int NUM_MODULES = 4;

    private static final int defaultPidSlotId = 0;
    private static final int secondaryPidSlotId = 1;
    private static final int FRAME_PERIOD_MS = 5;

    private static final LoggingKey[] ENCODER_ANGLE_LOGGING_KEYS = { LoggingKey.DriveTrainAbsoluteEncoderAngle1, LoggingKey.DriveTrainAbsoluteEncoderAngle2, LoggingKey.DriveTrainAbsoluteEncoderAngle3, LoggingKey.DriveTrainAbsoluteEncoderAngle4 };
    private static final LoggingKey[] DRIVE_VELOCITY_LOGGING_KEYS = { LoggingKey.DriveTrainDriveVelocity1, LoggingKey.DriveTrainDriveVelocity2, LoggingKey.DriveTrainDriveVelocity3, LoggingKey.DriveTrainDriveVelocity4 };
    private static final LoggingKey[] DRIVE_POSITION_LOGGING_KEYS = { LoggingKey.DriveTrainDrivePosition1, LoggingKey.DriveTrainDrivePosition2, LoggingKey.DriveTrainDrivePosition3, LoggingKey.DriveTrainDrivePosition4 };
    private static final LoggingKey[] DRIVE_ERROR_LOGGING_KEYS = { LoggingKey.DriveTrainDriveError1, LoggingKey.DriveTrainDriveError2, LoggingKey.DriveTrainDriveError3, LoggingKey.DriveTrainDriveError4 };
    private static final LoggingKey[] STEER_VELOCITY_LOGGING_KEYS = { LoggingKey.DriveTrainSteerVelocity1, LoggingKey.DriveTrainSteerVelocity2, LoggingKey.DriveTrainSteerVelocity3, LoggingKey.DriveTrainSteerVelocity4 };
    private static final LoggingKey[] STEER_POSITION_LOGGING_KEYS = { LoggingKey.DriveTrainSteerPosition1, LoggingKey.DriveTrainSteerPosition2, LoggingKey.DriveTrainSteerPosition3, LoggingKey.DriveTrainSteerPosition4 };
    private static final LoggingKey[] STEER_ANGLE_LOGGING_KEYS = { LoggingKey.DriveTrainSteerAngle1, LoggingKey.DriveTrainSteerAngle2, LoggingKey.DriveTrainSteerAngle3, LoggingKey.DriveTrainSteerAngle4 };
    private static final LoggingKey[] STEER_ERROR_LOGGING_KEYS = { LoggingKey.DriveTrainSteerError1, LoggingKey.DriveTrainSteerError2, LoggingKey.DriveTrainSteerError3, LoggingKey.DriveTrainSteerError4 };
    private static final LoggingKey[] DRIVE_GOAL_LOGGING_KEYS = { LoggingKey.DriveTrainDriveVelocityGoal1, LoggingKey.DriveTrainDriveVelocityGoal2, LoggingKey.DriveTrainDriveVelocityGoal3, LoggingKey.DriveTrainDriveVelocityGoal4 };
    private static final LoggingKey[] STEER_GOAL_LOGGING_KEYS = { LoggingKey.DriveTrainSteerPositionGoal1, LoggingKey.DriveTrainSteerPositionGoal2, LoggingKey.DriveTrainSteerPositionGoal3, LoggingKey.DriveTrainSteerPositionGoal4 };

    private static final AnalogOperation[] STEER_SETPOINT_OPERATIONS = new AnalogOperation[] { AnalogOperation.DriveTrainPositionSteer1, AnalogOperation.DriveTrainPositionSteer2, AnalogOperation.DriveTrainPositionSteer3, AnalogOperation.DriveTrainPositionSteer4 };
    private static final AnalogOperation[] DRIVE_SETPOINT_OPERATIONS = new AnalogOperation[] { AnalogOperation.DriveTrainPositionDrive1, AnalogOperation.DriveTrainPositionDrive2, AnalogOperation.DriveTrainPositionDrive3, AnalogOperation.DriveTrainPositionDrive4 };

    // the x offsets of the swerve modules from the default center of rotation
    private final double[] moduleOffsetX;

    // the y offsets of the swerve modules from the default center of rotation
    private final double[] moduleOffsetY;

    private final double[] drivetrainSteerMotorAbsoluteOffsets;;

    private final IDriver driver;
    private final ILogger logger;
    private final ITimer timer;

    private final PigeonManager pigeonManager;

    private final ITalonFX[] steerMotors;
    private final ITalonFX[] driveMotors;
    private final IAnalogInput[] absoluteEncoders;

    private final PIDHandler omegaPID;
    private final boolean[] isDirectionSwapped;
    private final PIDHandler pathOmegaPID;
    private final PIDHandler pathXOffsetPID;
    private final PIDHandler pathYOffsetPID;

    private final double[] driveVelocities;
    private final double[] drivePositions;
    private final double[] driveErrors;
    private final double[] steerVelocities;
    private final double[] steerPositions;
    private final double[] steerAngles;
    private final double[] steerErrors;
    private final double[] encoderVoltages;
    private final double[] encoderAngles;

    private final Setpoint[] result;

    private boolean firstRun;

    private boolean fieldOriented;
    private boolean maintainOrientation;
    private boolean updatedOrientation;
    private double desiredYaw;

    private double time;
    private double angle;
    private double xPosition;
    private double yPosition;
    private double deltaT;

    private double robotYaw;

    @Inject
    public DriveTrainMechanism(
        IDriver driver,
        LoggingManager logger,
        IRobotProvider provider,
        PigeonManager pigeonManager,
        ITimer timer)
    {
        this.driver = driver;
        this.logger = logger;
        this.timer = timer;

        this.pigeonManager = pigeonManager;

        this.steerMotors = new ITalonFX[DriveTrainMechanism.NUM_MODULES];
        this.driveMotors = new ITalonFX[DriveTrainMechanism.NUM_MODULES];
        this.absoluteEncoders = new IAnalogInput[DriveTrainMechanism.NUM_MODULES];

        this.moduleOffsetX =
            new double[]
            {
                -HardwareConstants.DRIVETRAIN_HORIZONTAL_WHEEL_CENTER_DISTANCE, // module 1 (front-right)
                HardwareConstants.DRIVETRAIN_HORIZONTAL_WHEEL_CENTER_DISTANCE, // module 2 (front-left)
                HardwareConstants.DRIVETRAIN_HORIZONTAL_WHEEL_CENTER_DISTANCE, // module 3 (back-left)
                -HardwareConstants.DRIVETRAIN_HORIZONTAL_WHEEL_CENTER_DISTANCE, // module 4 (back-right)
            };

        this.moduleOffsetY =
            new double[]
            {
                -HardwareConstants.DRIVETRAIN_VERTICAL_WHEEL_CENTER_DISTANCE, // module 1 (front-right)
                -HardwareConstants.DRIVETRAIN_VERTICAL_WHEEL_CENTER_DISTANCE, // module 2 (front-left)
                HardwareConstants.DRIVETRAIN_VERTICAL_WHEEL_CENTER_DISTANCE, // module 3 (back-left)
                HardwareConstants.DRIVETRAIN_VERTICAL_WHEEL_CENTER_DISTANCE, // module 4 (back-right)
            };

        this.drivetrainSteerMotorAbsoluteOffsets =
            new double[]
            {
                TuningConstants.DRIVETRAIN_STEER_MOTOR1_ABSOLUTE_OFFSET,
                TuningConstants.DRIVETRAIN_STEER_MOTOR2_ABSOLUTE_OFFSET,
                TuningConstants.DRIVETRAIN_STEER_MOTOR3_ABSOLUTE_OFFSET,
                TuningConstants.DRIVETRAIN_STEER_MOTOR4_ABSOLUTE_OFFSET,
            };

        int[] driveMotorCanIds =
            new int[]
            {
                ElectronicsConstants.DRIVETRAIN_DRIVE_MOTOR_1_CAN_ID,
                ElectronicsConstants.DRIVETRAIN_DRIVE_MOTOR_2_CAN_ID,
                ElectronicsConstants.DRIVETRAIN_DRIVE_MOTOR_3_CAN_ID,
                ElectronicsConstants.DRIVETRAIN_DRIVE_MOTOR_4_CAN_ID
            };

        int[] steerMotorCanIds =
            new int[]
            {
                ElectronicsConstants.DRIVETRAIN_STEER_MOTOR_1_CAN_ID,
                ElectronicsConstants.DRIVETRAIN_STEER_MOTOR_2_CAN_ID,
                ElectronicsConstants.DRIVETRAIN_STEER_MOTOR_3_CAN_ID,
                ElectronicsConstants.DRIVETRAIN_STEER_MOTOR_4_CAN_ID
            };

        int[] absoluteEncoderAnalogInputs =
            new int[]
            {
                ElectronicsConstants.DRIVETRAIN_ABSOLUTE_ENCODER_1_ANALOG_INPUT,
                ElectronicsConstants.DRIVETRAIN_ABSOLUTE_ENCODER_2_ANALOG_INPUT,
                ElectronicsConstants.DRIVETRAIN_ABSOLUTE_ENCODER_3_ANALOG_INPUT,
                ElectronicsConstants.DRIVETRAIN_ABSOLUTE_ENCODER_4_ANALOG_INPUT
            };

        boolean[] driveMotorInvertOutputs =
            new boolean[]
            {
                HardwareConstants.DRIVETRAIN_DRIVE_MOTOR1_INVERT_OUTPUT,
                HardwareConstants.DRIVETRAIN_DRIVE_MOTOR2_INVERT_OUTPUT,
                HardwareConstants.DRIVETRAIN_DRIVE_MOTOR3_INVERT_OUTPUT,
                HardwareConstants.DRIVETRAIN_DRIVE_MOTOR4_INVERT_OUTPUT
            };

        boolean[] driveMotorInvertSensors =
            new boolean[]
            {
                HardwareConstants.DRIVETRAIN_DRIVE_MOTOR1_INVERT_SENSOR,
                HardwareConstants.DRIVETRAIN_DRIVE_MOTOR2_INVERT_SENSOR,
                HardwareConstants.DRIVETRAIN_DRIVE_MOTOR3_INVERT_SENSOR,
                HardwareConstants.DRIVETRAIN_DRIVE_MOTOR4_INVERT_SENSOR
            };

        boolean[] steerMotorInvertOutputs =
            new boolean[]
            {
                HardwareConstants.DRIVETRAIN_STEER_MOTOR1_INVERT_OUTPUT,
                HardwareConstants.DRIVETRAIN_STEER_MOTOR2_INVERT_OUTPUT,
                HardwareConstants.DRIVETRAIN_STEER_MOTOR3_INVERT_OUTPUT,
                HardwareConstants.DRIVETRAIN_STEER_MOTOR4_INVERT_OUTPUT
            };

        boolean[] steerMotorInvertSensors =
            new boolean[]
            {
                HardwareConstants.DRIVETRAIN_STEER_MOTOR1_INVERT_SENSOR,
                HardwareConstants.DRIVETRAIN_STEER_MOTOR2_INVERT_SENSOR,
                HardwareConstants.DRIVETRAIN_STEER_MOTOR3_INVERT_SENSOR,
                HardwareConstants.DRIVETRAIN_STEER_MOTOR4_INVERT_SENSOR
            };

        double[][] driveMotorVelocityKPIDFs =
            new double[][]
            {
                new double[]
                {
                    TuningConstants.DRIVETRAIN_DRIVE_MOTOR1_VELOCITY_PID_KP,
                    TuningConstants.DRIVETRAIN_DRIVE_MOTOR1_VELOCITY_PID_KI,
                    TuningConstants.DRIVETRAIN_DRIVE_MOTOR1_VELOCITY_PID_KD,
                    TuningConstants.DRIVETRAIN_DRIVE_MOTOR1_VELOCITY_PID_KF,
                },
                new double[]
                {
                    TuningConstants.DRIVETRAIN_DRIVE_MOTOR2_VELOCITY_PID_KP,
                    TuningConstants.DRIVETRAIN_DRIVE_MOTOR2_VELOCITY_PID_KI,
                    TuningConstants.DRIVETRAIN_DRIVE_MOTOR2_VELOCITY_PID_KD,
                    TuningConstants.DRIVETRAIN_DRIVE_MOTOR2_VELOCITY_PID_KF,
                },
                new double[]
                {
                    TuningConstants.DRIVETRAIN_DRIVE_MOTOR3_VELOCITY_PID_KP,
                    TuningConstants.DRIVETRAIN_DRIVE_MOTOR3_VELOCITY_PID_KI,
                    TuningConstants.DRIVETRAIN_DRIVE_MOTOR3_VELOCITY_PID_KD,
                    TuningConstants.DRIVETRAIN_DRIVE_MOTOR3_VELOCITY_PID_KF,
                },
                new double[]
                {
                    TuningConstants.DRIVETRAIN_DRIVE_MOTOR4_VELOCITY_PID_KP,
                    TuningConstants.DRIVETRAIN_DRIVE_MOTOR4_VELOCITY_PID_KI,
                    TuningConstants.DRIVETRAIN_DRIVE_MOTOR4_VELOCITY_PID_KD,
                    TuningConstants.DRIVETRAIN_DRIVE_MOTOR4_VELOCITY_PID_KF,
                },
            };

        double[][] driveMotorPositionKPIDFs =
            new double[][]
            {
                new double[]
                {
                    TuningConstants.DRIVETRAIN_DRIVE_MOTOR1_POSITION_PID_KP,
                    TuningConstants.DRIVETRAIN_DRIVE_MOTOR1_POSITION_PID_KI,
                    TuningConstants.DRIVETRAIN_DRIVE_MOTOR1_POSITION_PID_KD,
                    TuningConstants.DRIVETRAIN_DRIVE_MOTOR1_POSITION_PID_KF,
                },
                new double[]
                {
                    TuningConstants.DRIVETRAIN_DRIVE_MOTOR2_POSITION_PID_KP,
                    TuningConstants.DRIVETRAIN_DRIVE_MOTOR2_POSITION_PID_KI,
                    TuningConstants.DRIVETRAIN_DRIVE_MOTOR2_POSITION_PID_KD,
                    TuningConstants.DRIVETRAIN_DRIVE_MOTOR2_POSITION_PID_KF,
                },
                new double[]
                {
                    TuningConstants.DRIVETRAIN_DRIVE_MOTOR3_POSITION_PID_KP,
                    TuningConstants.DRIVETRAIN_DRIVE_MOTOR3_POSITION_PID_KI,
                    TuningConstants.DRIVETRAIN_DRIVE_MOTOR3_POSITION_PID_KD,
                    TuningConstants.DRIVETRAIN_DRIVE_MOTOR3_POSITION_PID_KF,
                },
                new double[]
                {
                    TuningConstants.DRIVETRAIN_DRIVE_MOTOR4_POSITION_PID_KP,
                    TuningConstants.DRIVETRAIN_DRIVE_MOTOR4_POSITION_PID_KI,
                    TuningConstants.DRIVETRAIN_DRIVE_MOTOR4_POSITION_PID_KD,
                    TuningConstants.DRIVETRAIN_DRIVE_MOTOR4_POSITION_PID_KF,
                },
            };

        double[][] steerMotorPositionKPIDFs =
            new double[][]
            {
                new double[]
                {
                    TuningConstants.DRIVETRAIN_STEER_MOTOR1_POSITION_PID_KP,
                    TuningConstants.DRIVETRAIN_STEER_MOTOR1_POSITION_PID_KI,
                    TuningConstants.DRIVETRAIN_STEER_MOTOR1_POSITION_PID_KD,
                    TuningConstants.DRIVETRAIN_STEER_MOTOR1_POSITION_PID_KF,
                },
                new double[]
                {
                    TuningConstants.DRIVETRAIN_STEER_MOTOR2_POSITION_PID_KP,
                    TuningConstants.DRIVETRAIN_STEER_MOTOR2_POSITION_PID_KI,
                    TuningConstants.DRIVETRAIN_STEER_MOTOR2_POSITION_PID_KD,
                    TuningConstants.DRIVETRAIN_STEER_MOTOR2_POSITION_PID_KF,
                },
                new double[]
                {
                    TuningConstants.DRIVETRAIN_STEER_MOTOR3_POSITION_PID_KP,
                    TuningConstants.DRIVETRAIN_STEER_MOTOR3_POSITION_PID_KI,
                    TuningConstants.DRIVETRAIN_STEER_MOTOR3_POSITION_PID_KD,
                    TuningConstants.DRIVETRAIN_STEER_MOTOR3_POSITION_PID_KF,
                },
                new double[]
                {
                    TuningConstants.DRIVETRAIN_STEER_MOTOR4_POSITION_PID_KP,
                    TuningConstants.DRIVETRAIN_STEER_MOTOR4_POSITION_PID_KI,
                    TuningConstants.DRIVETRAIN_STEER_MOTOR4_POSITION_PID_KD,
                    TuningConstants.DRIVETRAIN_STEER_MOTOR4_POSITION_PID_KF,
                },
            };

        for (int i = 0; i < DriveTrainMechanism.NUM_MODULES; i++)
        {
            this.driveMotors[i] = provider.getTalonFX(driveMotorCanIds[i]);
            this.driveMotors[i].setNeutralMode(MotorNeutralMode.Brake);
            this.driveMotors[i].setSensorType(TalonXFeedbackDevice.IntegratedSensor);
            this.driveMotors[i].setFeedbackFramePeriod(DriveTrainMechanism.FRAME_PERIOD_MS);
            this.driveMotors[i].setPIDFFramePeriod(DriveTrainMechanism.FRAME_PERIOD_MS);
            this.driveMotors[i].setInvertOutput(driveMotorInvertOutputs[i]);
            this.driveMotors[i].setInvertSensor(driveMotorInvertSensors[i]);
            this.driveMotors[i].configureVelocityMeasurements(10, 32);
            this.driveMotors[i].setPIDF(
                driveMotorVelocityKPIDFs[i][0],
                driveMotorVelocityKPIDFs[i][1],
                driveMotorVelocityKPIDFs[i][2],
                driveMotorVelocityKPIDFs[i][3],
                DriveTrainMechanism.defaultPidSlotId);
            this.driveMotors[i].setPIDF(
                driveMotorPositionKPIDFs[i][0],
                driveMotorPositionKPIDFs[i][1],
                driveMotorPositionKPIDFs[i][2],
                driveMotorPositionKPIDFs[i][3],
                DriveTrainMechanism.secondaryPidSlotId);
            this.driveMotors[i].setVoltageCompensation(
                TuningConstants.DRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION_ENABLED,
                TuningConstants.DRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION);
            this.driveMotors[i].setSupplyCurrentLimit(
                TuningConstants.DRIVETRAIN_DRIVE_SUPPLY_CURRENT_LIMITING_ENABLED,
                TuningConstants.DRIVETRAIN_DRIVE_SUPPLY_CURRENT_MAX,
                TuningConstants.DRIVETRAIN_DRIVE_SUPPLY_TRIGGER_CURRENT,
                TuningConstants.DRIVETRAIN_DRIVE_SUPPLY_TRIGGER_DURATION);
            this.driveMotors[i].setControlMode(TalonXControlMode.Velocity);
            this.driveMotors[i].setSelectedSlot(DriveTrainMechanism.defaultPidSlotId);

            this.steerMotors[i] = provider.getTalonFX(steerMotorCanIds[i]);
            this.steerMotors[i].setInvertOutput(steerMotorInvertOutputs[i]);
            this.steerMotors[i].setInvertSensor(steerMotorInvertSensors[i]);
            this.steerMotors[i].setNeutralMode(MotorNeutralMode.Brake);
            this.steerMotors[i].setSensorType(TalonXFeedbackDevice.IntegratedSensor);
            this.steerMotors[i].setPIDF(
                steerMotorPositionKPIDFs[i][0],
                steerMotorPositionKPIDFs[i][1],
                steerMotorPositionKPIDFs[i][2],
                steerMotorPositionKPIDFs[i][3],
                DriveTrainMechanism.defaultPidSlotId);
            this.steerMotors[i].setVoltageCompensation(
                TuningConstants.DRIVETRAIN_STEER_VOLTAGE_COMPENSATION_ENABLED,
                TuningConstants.DRIVETRAIN_STEER_VOLTAGE_COMPENSATION);
            this.steerMotors[i].setSupplyCurrentLimit(
                TuningConstants.DRIVETRAIN_STEER_SUPPLY_CURRENT_LIMITING_ENABLED,
                TuningConstants.DRIVETRAIN_STEER_SUPPLY_CURRENT_MAX,
                TuningConstants.DRIVETRAIN_STEER_SUPPLY_TRIGGER_CURRENT,
                TuningConstants.DRIVETRAIN_STEER_SUPPLY_TRIGGER_DURATION);
            this.steerMotors[i].setControlMode(TalonXControlMode.Position);
            this.steerMotors[i].setSelectedSlot(DriveTrainMechanism.defaultPidSlotId);

            this.absoluteEncoders[i] = provider.getAnalogInput(absoluteEncoderAnalogInputs[i]);
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

        this.result = new Setpoint[DriveTrainMechanism.NUM_MODULES];
        for (int i = 0; i < DriveTrainMechanism.NUM_MODULES; i++)
        {
            this.result[i] = new Setpoint();
        }

        this.time = 0.0;
        this.angle = 0.0;
        this.xPosition = 0.0;
        this.yPosition = 0.0;

        this.firstRun = TuningConstants.DRIVETRAIN_RESET_ON_ROBOT_START;
        this.fieldOriented = false;
        this.maintainOrientation = true;
        this.updatedOrientation = false;
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

            this.logger.logNumber(DriveTrainMechanism.DRIVE_VELOCITY_LOGGING_KEYS[i], this.driveVelocities[i]);
            this.logger.logNumber(DriveTrainMechanism.DRIVE_POSITION_LOGGING_KEYS[i], this.drivePositions[i]);
            this.logger.logNumber(DriveTrainMechanism.DRIVE_ERROR_LOGGING_KEYS[i], this.driveErrors[i]);
            this.logger.logNumber(DriveTrainMechanism.STEER_VELOCITY_LOGGING_KEYS[i], this.steerVelocities[i]);
            this.logger.logNumber(DriveTrainMechanism.STEER_POSITION_LOGGING_KEYS[i], this.steerPositions[i]);
            this.logger.logNumber(DriveTrainMechanism.STEER_ANGLE_LOGGING_KEYS[i], this.steerAngles[i]);
            this.logger.logNumber(DriveTrainMechanism.STEER_ERROR_LOGGING_KEYS[i], this.steerErrors[i]);
            this.logger.logNumber(DriveTrainMechanism.ENCODER_ANGLE_LOGGING_KEYS[i], this.encoderAngles[i]);
        }

        double prevYaw = this.robotYaw;
        double prevTime = this.time;
        this.robotYaw = this.pigeonManager.getAngle();
        this.time = this.timer.get();

        this.deltaT = this.time - prevTime;
        if (this.deltaT <= 0.0)
        {
            // keep this positive...
            this.deltaT = 0.001;
        }

        if (TuningConstants.DRIVETRAIN_USE_ODOMETRY)
        {
            double deltaNavxYaw = (this.robotYaw - prevYaw) / this.deltaT;
            this.calculateOdometry(deltaNavxYaw);
            this.logger.logNumber(LoggingKey.DriveTrainXPosition, this.xPosition);
            this.logger.logNumber(LoggingKey.DriveTrainYPosition, this.yPosition);
            this.logger.logNumber(LoggingKey.DriveTrainAngle, this.angle);
        }
    }

    @Override
    public void update()
    {
        if (this.driver.getDigital(DigitalOperation.DriveTrainEnableFieldOrientation))
        {
            this.fieldOriented = true;
            this.desiredYaw = this.robotYaw;
        }

        if (this.driver.getDigital(DigitalOperation.DriveTrainDisableFieldOrientation) ||
            this.pigeonManager.getIsConnected())
        {
            this.fieldOriented = false;
        }

        if (this.driver.getDigital(DigitalOperation.DriveTrainEnableMaintainDirectionMode))
        {
            this.maintainOrientation = true;
        }

        if (this.driver.getDigital(DigitalOperation.DriveTrainEnableMaintainDirectionMode) ||
            !this.pigeonManager.getIsConnected())
        {
            this.maintainOrientation = false;
        }

        if (this.driver.getDigital(DigitalOperation.PositionResetFieldOrientation))
        {
            this.robotYaw = this.pigeonManager.getAngle();
            this.desiredYaw = this.robotYaw;
        }

        if (this.firstRun || this.driver.getDigital(DigitalOperation.DriveTrainReset))
        {
            for (int i = 0; i < DriveTrainMechanism.NUM_MODULES; i++)
            {
                this.driveMotors[i].setPosition(0);
                double angleDifference = (this.encoderAngles[i] - this.drivetrainSteerMotorAbsoluteOffsets[i]);
                double tickDifference = angleDifference * HardwareConstants.DRIVETRAIN_STEER_TICKS_PER_DEGREE;
                this.steerMotors[i].setPosition((int)tickDifference);

                this.drivePositions[i] = 0;
                this.steerPositions[i] = (int)tickDifference;
                this.steerAngles[i] = angleDifference % 360.0;
            }

            this.firstRun = false;
        }

        this.calculateSetpoints();
        for (int i = 0; i < DriveTrainMechanism.NUM_MODULES; i++)
        {
            Setpoint current = this.result[i];
            Double steerSetpoint = current.angle;
            Double driveVelocitySetpoint = current.driveVelocity;
            Double drivePositionSetpoint = current.drivePosition;

            TalonXControlMode driveControlMode = TalonXControlMode.Disabled;
            int driveDesiredPidSlotId = DriveTrainMechanism.defaultPidSlotId;
            double driveSetpoint = 0.0;
            if (driveVelocitySetpoint != null)
            {
                driveSetpoint = driveVelocitySetpoint;
                driveControlMode = TalonXControlMode.Velocity;
                driveDesiredPidSlotId = DriveTrainMechanism.defaultPidSlotId;
            }
            else if (drivePositionSetpoint != null)
            {
                driveSetpoint = drivePositionSetpoint;
                driveControlMode = TalonXControlMode.Position;
                driveDesiredPidSlotId = DriveTrainMechanism.secondaryPidSlotId;
            }

            this.logger.logNumber(DriveTrainMechanism.DRIVE_GOAL_LOGGING_KEYS[i], driveSetpoint);
            this.driveMotors[i].setControlMode(driveControlMode);
            if (driveControlMode != TalonXControlMode.Disabled)
            {
                this.driveMotors[i].set(driveSetpoint);
                this.driveMotors[i].setSelectedSlot(driveDesiredPidSlotId);
            }

            if (steerSetpoint != null)
            {
                this.logger.logNumber(DriveTrainMechanism.STEER_GOAL_LOGGING_KEYS[i], steerSetpoint);
                this.steerMotors[i].set(steerSetpoint);
            }
        }
    }

    @Override
    public void stop()
    {
        this.omegaPID.reset();
        this.pathOmegaPID.reset();
        this.pathXOffsetPID.reset();
        this.pathYOffsetPID.reset();
        for (int i = 0; i < DriveTrainMechanism.NUM_MODULES; i++)
        {
            this.driveMotors[i].stop();
            this.steerMotors[i].stop();
        }
    }

    public double[] getModuleTurnInPlaceAngles()
    {
        return new double[]
            {
                Helpers.atan2d(-this.moduleOffsetY[0], -this.moduleOffsetX[0]),
                Helpers.atan2d(-this.moduleOffsetY[1], -this.moduleOffsetX[1]),
                Helpers.atan2d(-this.moduleOffsetY[2], -this.moduleOffsetX[2]),
                Helpers.atan2d(-this.moduleOffsetY[3], -this.moduleOffsetX[3]),
            };
    }

    public double[] getDriveMotorPositions()
    {
        return new double[]
            {
                this.drivePositions[0],
                this.drivePositions[1],
                this.drivePositions[2],
                this.drivePositions[3],
            };
    }

    public Pose2d getPose()
    {
        return new Pose2d(this.xPosition, this.yPosition, this.robotYaw);
    }

    private void calculateSetpoints()
    {
        if (this.driver.getDigital(DigitalOperation.DriveTrainPositionMode))
        {
            for (int i = 0; i < DriveTrainMechanism.NUM_MODULES; i++)
            {
                this.result[i].driveVelocity = null;
                this.result[i].drivePosition = this.driver.getAnalog(DriveTrainMechanism.DRIVE_SETPOINT_OPERATIONS[i]);

                double moduleSteerPositionGoal = this.driver.getAnalog(DriveTrainMechanism.STEER_SETPOINT_OPERATIONS[i]);
                double currentAngle = this.steerPositions[i] * HardwareConstants.DRIVETRAIN_STEER_PULSE_DISTANCE;
                AnglePair anglePair = AnglePair.getClosestAngle(moduleSteerPositionGoal, currentAngle, true);
                moduleSteerPositionGoal = anglePair.getAngle() * TuningConstants.DRIVETRAIN_STEER_MOTOR_POSITION_PID_KS;
                this.isDirectionSwapped[i] = anglePair.getSwapDirection();

                this.result[i].angle = moduleSteerPositionGoal;
            }

            return;
        }

        // calculate center velocity and turn velocity based on our current control mode:
        double rotationCenterA;
        double rotationCenterB;

        // robot center velocity, in inches/sec
        double centerVelocityRight;
        double centerVelocityForward;

        // robot turn velocity, in rad/sec
        double omega;
        if (this.driver.getDigital(DigitalOperation.DriveTrainPathMode))
        {
            // path mode doesn't support rotation centers besides the robot center
            rotationCenterA = 0.0;
            rotationCenterB = 0.0;

            double xGoal = this.driver.getAnalog(AnalogOperation.DriveTrainPathXGoal);
            double yGoal = this.driver.getAnalog(AnalogOperation.DriveTrainPathYGoal);
            double angleGoal = this.driver.getAnalog(AnalogOperation.DriveTrainTurnAngleGoal);
            double angleReference = this.driver.getAnalog(AnalogOperation.DriveTrainTurnAngleReference);
            double xVelocityGoal = this.driver.getAnalog(AnalogOperation.DriveTrainPathXVelocityGoal);
            double yVelocityGoal = this.driver.getAnalog(AnalogOperation.DriveTrainPathYVelocityGoal);
            double angleVelocityGoal = this.driver.getAnalog(AnalogOperation.DriveTrainPathAngleVelocityGoal);

            omega = angleVelocityGoal * Helpers.DEGREES_TO_RADIANS;
            if (this.fieldOriented)
            {
                // add correction for x/y drift
                xVelocityGoal += this.pathXOffsetPID.calculatePosition(xGoal, this.xPosition);
                yVelocityGoal += this.pathYOffsetPID.calculatePosition(yGoal, this.yPosition);

                // convert velocity to be robot-oriented
                centerVelocityRight = Helpers.cosd(this.robotYaw) * xVelocityGoal + Helpers.sind(this.robotYaw) * yVelocityGoal;
                centerVelocityForward = Helpers.cosd(this.robotYaw) * yVelocityGoal - Helpers.sind(this.robotYaw) * xVelocityGoal;

                // add correction for angle drift
                AnglePair anglePair = AnglePair.getClosestAngle(angleGoal + angleReference, this.robotYaw, false);
                this.desiredYaw = anglePair.getAngle();

                this.logger.logNumber(LoggingKey.DriveTrainDesiredAngle, this.desiredYaw);
                omega += this.pathOmegaPID.calculatePosition(this.desiredYaw, this.robotYaw);
            }
            else
            {
                centerVelocityRight = xVelocityGoal;
                centerVelocityForward = yVelocityGoal;
            }
        }
        else
        {
            // get the center of rotation modifying control values
            rotationCenterA = this.driver.getAnalog(AnalogOperation.DriveTrainRotationA);
            rotationCenterB = this.driver.getAnalog(AnalogOperation.DriveTrainRotationB);

            // get the center velocity control values (could be field-oriented or robot-oriented center velocity)
            double centerVelocityRightRaw = this.driver.getAnalog(AnalogOperation.DriveTrainMoveRight);
            double centerVelocityForwardRaw = this.driver.getAnalog(AnalogOperation.DriveTrainMoveForward);

            if (this.fieldOriented)
            {
                centerVelocityRight = Helpers.cosd(this.robotYaw) * centerVelocityRightRaw + Helpers.sind(this.robotYaw) * centerVelocityForwardRaw;
                centerVelocityForward = Helpers.cosd(this.robotYaw) * centerVelocityForwardRaw - Helpers.sind(this.robotYaw) * centerVelocityRightRaw;

                boolean hadUpdatedOrientation = this.updatedOrientation;
                this.updatedOrientation = false;
                double yawGoal = this.driver.getAnalog(AnalogOperation.DriveTrainTurnAngleGoal);
                if (yawGoal != TuningConstants.MAGIC_NULL_VALUE)
                {
                    this.updatedOrientation = true;

                    AnglePair anglePair = AnglePair.getClosestAngle(yawGoal, this.robotYaw, false);
                    this.desiredYaw = anglePair.getAngle();
                }
                else
                {
                    double turnSpeed = this.driver.getAnalog(AnalogOperation.DriveTrainTurnSpeed);
                    if (turnSpeed != 0.0)
                    {
                        this.updatedOrientation = true;
                        if (!hadUpdatedOrientation)
                        {
                            this.desiredYaw = this.robotYaw;
                        }

                        this.desiredYaw += turnSpeed * TuningConstants.DRIVETRAIN_TURN_GOAL_VELOCITY;
                    }
                }

                if (this.maintainOrientation || this.updatedOrientation)
                {
                    if (!this.updatedOrientation &&
                        TuningConstants.DRIVETRAIN_TURN_APPROXIMATION != 0.0 &&
                        Helpers.WithinDelta(this.desiredYaw, this.robotYaw, TuningConstants.DRIVETRAIN_TURN_APPROXIMATION))
                    {
                        // don't turn aggressively if we are within a very small delta from our goal angle
                        omega = 0.0;
                    }
                    else
                    {
                        this.logger.logNumber(LoggingKey.DriveTrainDesiredAngle, this.desiredYaw);
                        omega = this.omegaPID.calculatePosition(this.desiredYaw, this.robotYaw);
                    }
                }
                else
                {
                    omega = 0.0;
                }
            }
            else
            {
                centerVelocityRight = centerVelocityRightRaw;
                centerVelocityForward = centerVelocityForwardRaw;
                omega = this.driver.getAnalog(AnalogOperation.DriveTrainTurnSpeed) * TuningConstants.DRIVETRAIN_TURN_SCALE;
            }
        }

        double maxModuleDriveVelocityGoal = 0.0;
        for (int i = 0; i < DriveTrainMechanism.NUM_MODULES; i++)
        {
            double moduleVelocityRight = centerVelocityRight + omega * (this.moduleOffsetY[i] + rotationCenterB);
            double moduleVelocityForward = centerVelocityForward - omega * (this.moduleOffsetX[i] + rotationCenterA);

            Double moduleSteerPositionGoal;
            double moduleDriveVelocityGoal;
            if (TuningConstants.DRIVETRAIN_SKIP_ANGLE_ON_ZERO_VELOCITY
                    && Helpers.WithinDelta(moduleVelocityRight, 0.0, TuningConstants.DRIVETRAIN_SKIP_ANGLE_ON_ZERO_DELTA)
                    && Helpers.WithinDelta(moduleVelocityForward, 0.0, TuningConstants.DRIVETRAIN_SKIP_ANGLE_ON_ZERO_DELTA))
            {
                moduleDriveVelocityGoal = 0.0;
                moduleSteerPositionGoal = null;
            }
            else
            {
                moduleDriveVelocityGoal = Math.sqrt(moduleVelocityRight * moduleVelocityRight + moduleVelocityForward * moduleVelocityForward);

                moduleSteerPositionGoal = Helpers.atan2d(-moduleVelocityRight, moduleVelocityForward);
                double currentAngle = this.steerPositions[i] * HardwareConstants.DRIVETRAIN_STEER_PULSE_DISTANCE;
                AnglePair anglePair = AnglePair.getClosestAngle(moduleSteerPositionGoal, currentAngle, true);
                moduleSteerPositionGoal = anglePair.getAngle() * TuningConstants.DRIVETRAIN_STEER_MOTOR_POSITION_PID_KS;
                this.isDirectionSwapped[i] = anglePair.getSwapDirection();

                if (maxModuleDriveVelocityGoal < moduleDriveVelocityGoal)
                {
                    maxModuleDriveVelocityGoal = moduleDriveVelocityGoal;
                }

                moduleDriveVelocityGoal *= HardwareConstants.DRIVETRAIN_DRIVE_INCHES_PER_SECOND_TO_MOTOR_VELOCITY;
                if (this.isDirectionSwapped[i])
                {
                    moduleDriveVelocityGoal *= -1.0;
                }
            }

            this.result[i].driveVelocity = moduleDriveVelocityGoal;
            this.result[i].drivePosition = null;
            this.result[i].angle = moduleSteerPositionGoal;
        }

        // rescale velocities based on max velocity percentage, if max velocity is exceeded for any module
        if (maxModuleDriveVelocityGoal > TuningConstants.DRIVETRAIN_MAX_VELOCITY)
        {
            // divide by percentage is interchangeable with multiply by inverse-percentage
            double invPercentage = TuningConstants.DRIVETRAIN_MAX_VELOCITY / maxModuleDriveVelocityGoal;
            for (int i = 0; i < DriveTrainMechanism.NUM_MODULES; i++)
            {
                this.result[i].driveVelocity *= invPercentage;
            }
        }
    }

    private void calculateOdometry(double deltaNavxYaw)
    {
        double navxOmega = deltaNavxYaw / this.deltaT; // in degrees

        double rightRobotVelocity;
        double forwardRobotVelocity;

        // calculate our right and forward velocities using an average of our various velocities and the angle.
        double rightRobotVelocity1 = -Helpers.sind(this.steerAngles[0]) * HardwareConstants.DRIVETRAIN_DRIVE_MOTOR_VELOCITY_TO_INCHES_PER_SECOND * this.driveVelocities[0];
        double rightRobotVelocity2 = -Helpers.sind(this.steerAngles[1]) * HardwareConstants.DRIVETRAIN_DRIVE_MOTOR_VELOCITY_TO_INCHES_PER_SECOND * this.driveVelocities[1];
        double rightRobotVelocity3 = -Helpers.sind(this.steerAngles[2]) * HardwareConstants.DRIVETRAIN_DRIVE_MOTOR_VELOCITY_TO_INCHES_PER_SECOND * this.driveVelocities[2];
        double rightRobotVelocity4 = -Helpers.sind(this.steerAngles[3]) * HardwareConstants.DRIVETRAIN_DRIVE_MOTOR_VELOCITY_TO_INCHES_PER_SECOND * this.driveVelocities[3];

        double forwardRobotVelocity1 = Helpers.cosd(this.steerAngles[0]) * HardwareConstants.DRIVETRAIN_DRIVE_MOTOR_VELOCITY_TO_INCHES_PER_SECOND * this.driveVelocities[0];
        double forwardRobotVelocity2 = Helpers.cosd(this.steerAngles[1]) * HardwareConstants.DRIVETRAIN_DRIVE_MOTOR_VELOCITY_TO_INCHES_PER_SECOND * this.driveVelocities[1];
        double forwardRobotVelocity3 = Helpers.cosd(this.steerAngles[2]) * HardwareConstants.DRIVETRAIN_DRIVE_MOTOR_VELOCITY_TO_INCHES_PER_SECOND * this.driveVelocities[2];
        double forwardRobotVelocity4 = Helpers.cosd(this.steerAngles[3]) * HardwareConstants.DRIVETRAIN_DRIVE_MOTOR_VELOCITY_TO_INCHES_PER_SECOND * this.driveVelocities[3];

        // rightRobotVelocity = (rightRobotVelocity1 + rightRobotVelocity2 + rightRobotVelocity3 + rightRobotVelocity4) / 4.0;
        // forwardRobotVelocity = (forwardRobotVelocity1 + forwardRobotVelocity2 + forwardRobotVelocity3 + forwardRobotVelocity4) / 4.0;

        double a = 0.5 * (-rightRobotVelocity3 - rightRobotVelocity4);
        double b = 0.5 * (-rightRobotVelocity1 - rightRobotVelocity2);
        double c = 0.5 * (forwardRobotVelocity1 + forwardRobotVelocity4);
        double d = 0.5 * (forwardRobotVelocity2 + forwardRobotVelocity3);

        double omegaRadians1 = (b - a) / HardwareConstants.DRIVETRAIN_VERTICAL_WHEEL_SEPERATION_DISTANCE;
        double omegaRadians2 = (c - d) / HardwareConstants.DRIVETRAIN_HORIZONTAL_WHEEL_SEPERATION_DISTANCE;
        double omegaRadians = (omegaRadians1 + omegaRadians2) / 2.0;

        double rightRobotVelocityA = omegaRadians * HardwareConstants.DRIVETRAIN_HORIZONTAL_WHEEL_CENTER_DISTANCE + a;
        double rightRobotVelocityB = -omegaRadians * HardwareConstants.DRIVETRAIN_HORIZONTAL_WHEEL_CENTER_DISTANCE + b;
        rightRobotVelocity = -(rightRobotVelocityA + rightRobotVelocityB) / 2.0;

        double forwardRobotVelocityA = omegaRadians * HardwareConstants.DRIVETRAIN_VERTICAL_WHEEL_CENTER_DISTANCE + c;
        double forwardRobotVelocityB = -omegaRadians * HardwareConstants.DRIVETRAIN_VERTICAL_WHEEL_CENTER_DISTANCE + d;
        forwardRobotVelocity = (forwardRobotVelocityA + forwardRobotVelocityB) / 2.0;

        this.angle += omegaRadians * Helpers.RADIANS_TO_DEGREES * this.deltaT;

        double rightFieldVelocity = rightRobotVelocity * Helpers.cosd(this.robotYaw) - forwardRobotVelocity * Helpers.sind(this.robotYaw);
        double forwardFieldVelocity = rightRobotVelocity * Helpers.sind(this.robotYaw) + forwardRobotVelocity * Helpers.cosd(this.robotYaw);
        this.xPosition += rightFieldVelocity * this.deltaT;
        this.yPosition += forwardFieldVelocity * this.deltaT;
    }

    /**
     * Basic structure to hold an angle/drive pair
     */
    private class Setpoint
    {
        public Double angle;
        public Double driveVelocity;
        public Double drivePosition;

        public Setpoint()
        {
        }
    }
}