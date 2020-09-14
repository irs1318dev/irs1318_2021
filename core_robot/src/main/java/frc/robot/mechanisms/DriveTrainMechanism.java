package frc.robot.mechanisms;

import java.lang.invoke.InjectedProfile;

import javax.inject.Singleton;

import frc.robot.*;
import frc.robot.common.*;
import frc.robot.common.robotprovider.*;
import frc.robot.driver.*;
import frc.robot.driver.common.Driver;

import com.google.inject.Inject;

@Singleton
public class DriveTrainMechanism implements IMechanism {

    private static final int pidSlotId = 0;
    private static final int FRAME_PERIOD_MS = 5;

    private static final double POWERLEVEL_MIN = -1.0;
    private static final double POWERLEVEL_MAX = 1.0;

    private final ILogger logger;
    private final ITimer timer;

    private final ITalonFX driveMotor;
    private final ITalonFX angleMotor;

    private Driver driver;

    private PIDHandler drivePID;
    private PIDHandler anglePID;

    private boolean usePID;
    private boolean useBrakeMode;

    private double driveVelocity;
    private double driveError;
    private int drivePosition;
    private double angleVelocity;
    private double angleError;
    private int anglePosition;

    @Inject
    public DriveTrainMechanism(
    LoggingManager logger,
    IRobotProvider provider,
    ITimer timer) {

        this.logger = logger;
        this.timer = timer;

        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        this.angleMotor = provider.getTalonSRX(ElectronicsConstants.ANGLE_MOTOR_CAN_ID);
        this.angleMotor.setInvertOutput(HardwareConstants.ANGLE_MOTOR_INVERT_OUTPUT);
        this.angleMotor.setInvertSensor(HardwareConstants.ANGLE_MOTOR_INVERT_SENSOR);
        this.angleMotor.setNeutralMode(MotorNeutralMode.Brake);
        this.angleMotor.setSensorType(AnalogInput.);
        this.angleMotor.setPosition(0);
        this.angleMotor.setControlMode(TalonSRXControlMode.Position);
        this.angleMotor.setPIDF(
            TuningConstants.ANGLE_MOTOR_POSITION_PID_KP,
            TuningConstants.ANGLE_MOTOR_POSITION_PID_KI,
            TuningConstants.ANGLE_MOTOR_POSITION_PID_KD,
            TuningConstants.ANGLE_MOTOR_POSITION_PID_KF,
            DriveTrainMechanism.slotId);
    }
}
