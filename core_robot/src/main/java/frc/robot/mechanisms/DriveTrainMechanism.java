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
    private final ITalonFX steeringMotor;

    private Driver driver;

    private PIDHandler drivePID;
    private PIDHandler steeringPID;

    private boolean usePID;
    private boolean useBrakeMode;

    private double driveVelocity;
    private double driveError;
    private int drivePosition;
    private double steeringVelocity;
    private double steeringError;
    private int steeringPosition;

    @Inject
    public DriveTrainMechanism(
    LoggingManager logger,
    IRobotProvider provider,
    ITimer timer) {

        this.logger = logger;
        this.timer = timer;

        this.turret = provider.getTalonSRX(ElectronicsConstants.POWERCELL_TURRET_MOTOR_CAN_ID);
        this.turret.setInvertOutput(HardwareConstants.POWERCELL_TURRET_INVERT_OUTPUT);
        this.turret.setInvertSensor(HardwareConstants.POWERCELL_TURRET_INVERT_SENSOR);
        this.turret.setNeutralMode(MotorNeutralMode.Brake);
        this.turret.setSensorType(TalonXFeedbackDevice.QuadEncoder);
        this.turret.setPosition(0);
        this.turret.setForwardLimitSwitch(TuningConstants.POWERCELL_TURRET_FORWARD_LIMIT_SWITCH_ENABLED, TuningConstants.POWERCELL_TURRET_FORWARD_LIMIT_SWITCH_NORMALLY_OPEN);
        this.turret.setReverseLimitSwitch(TuningConstants.POWERCELL_TURRET_REVERSE_LIMIT_SWITCH_ENABLED, TuningConstants.POWERCELL_TURRET_REVERSE_LIMIT_SWITCH_NORMALLY_OPEN);
        this.turret.setControlMode(TalonSRXControlMode.Position);
        this.turret.setPIDF(
            TuningConstants.POWERCELL_TURRET_POSITION_PID_KP,
            TuningConstants.POWERCELL_TURRET_POSITION_PID_KI,
            TuningConstants.POWERCELL_TURRET_POSITION_PID_KD,
            TuningConstants.POWERCELL_TURRET_POSITION_PID_KF,
            PowerCellMechanism.slotId);
    }
}
