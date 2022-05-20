package frc.robot.common.robotprovider;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

public class TalonFXWrapper implements ITalonFX
{
    private static final int pidIdx = 0;
    private static final int timeoutMS = 10;

    final TalonFX wrappedObject;

    private ControlMode controlMode;

    public TalonFXWrapper(int deviceNumber)
    {
        this.wrappedObject = new TalonFX(deviceNumber);
        this.controlMode = ControlMode.PercentOutput;
    }

    public TalonFXWrapper(int deviceNumber, String canbus)
    {
        this.wrappedObject = new TalonFX(deviceNumber, canbus);
        this.controlMode = ControlMode.PercentOutput;
    }

    public void set(double value)
    {
        this.wrappedObject.set(this.controlMode, value);
    }

    public void follow(ITalonSRX talonSRX)
    {
        this.wrappedObject.follow(((TalonSRXWrapper)talonSRX).wrappedObject);
    }

    public void follow(ITalonFX talonFX)
    {
        this.wrappedObject.follow(((TalonFXWrapper)talonFX).wrappedObject);
    }

    public void follow(IVictorSPX victorSPX)
    {
        this.wrappedObject.follow(((VictorSPXWrapper)victorSPX).wrappedObject);
    }

    public void setControlMode(TalonXControlMode mode)
    {
        if (mode == TalonXControlMode.PercentOutput)
        {
            this.controlMode = ControlMode.PercentOutput;
        }
        else if (mode == TalonXControlMode.Disabled)
        {
            this.controlMode = ControlMode.Disabled;
        }
        else if (mode == TalonXControlMode.Follower)
        {
            this.controlMode = ControlMode.Follower;
        }
        else if (mode == TalonXControlMode.Position)
        {
            this.controlMode = ControlMode.Position;
        }
        else if (mode == TalonXControlMode.MotionMagicPosition)
        {
            this.controlMode = ControlMode.MotionMagic;
        }
        else if (mode == TalonXControlMode.Velocity)
        {
            this.controlMode = ControlMode.Velocity;
        }
    }

    public void setSensorType(TalonXFeedbackDevice feedbackDevice)
    {
        FeedbackDevice device;
        if (feedbackDevice == TalonXFeedbackDevice.QuadEncoder)
        {
            device = FeedbackDevice.QuadEncoder;
        }
        else if (feedbackDevice == TalonXFeedbackDevice.PulseWidthEncodedPosition)
        {
            device = FeedbackDevice.PulseWidthEncodedPosition;
        }
        else if (feedbackDevice == TalonXFeedbackDevice.IntegratedSensor)
        {
            device = FeedbackDevice.IntegratedSensor;
        }
        else
        {
            return;
        }

        CTREErrorCodeHelper.printError(
            this.wrappedObject.configSelectedFeedbackSensor(device, TalonFXWrapper.pidIdx, 0),
            "TalonFX.configSelectedFeedbackSensor");
    }

    public void setGeneralFramePeriod(int periodMS)
    {
        CTREErrorCodeHelper.printError(
            this.wrappedObject.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, periodMS, TalonFXWrapper.timeoutMS),
            "TalonFX.setGeneralFramePeriod");
    }

    public void setFeedbackFramePeriod(int periodMS)
    {
        CTREErrorCodeHelper.printError(
            this.wrappedObject.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, periodMS, TalonFXWrapper.timeoutMS),
            "TalonFX.setFeedbackFramePeriod");
    }

    public void setPIDFFramePeriod(int periodMS)
    {
        CTREErrorCodeHelper.printError(
            this.wrappedObject.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, periodMS, TalonFXWrapper.timeoutMS),
            "TalonFX.setPIDFFramePeriod");
    }

    public void configureVelocityMeasurements(int periodMS, int windowSize)
    {
        CTREErrorCodeHelper.printError(
            this.wrappedObject.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.valueOf(periodMS), TalonFXWrapper.timeoutMS),
            "TalonFX.configureVelocityMeasurementPeriod");
        CTREErrorCodeHelper.printError(
            this.wrappedObject.configVelocityMeasurementWindow(windowSize, TalonFXWrapper.timeoutMS),
            "TalonFX.configureVelocityMeasurementWindow");
    }

    public void configureAllowableClosedloopError(int slotId, int error)
    {
        CTREErrorCodeHelper.printError(
            this.wrappedObject.configAllowableClosedloopError(slotId, error, TalonFXWrapper.timeoutMS),
            "TalonFX.configureAllowableClosedloopError");
    }

    public void setSelectedSlot(int slotId)
    {
        this.wrappedObject.selectProfileSlot(slotId, TalonFXWrapper.pidIdx);
    }

    public void setPIDF(double p, double i, double d, double f, int slotId)
    {
        CTREErrorCodeHelper.printError(
            this.wrappedObject.config_kP(slotId, p, TalonFXWrapper.timeoutMS),
            "TalonFX.setPIDF_kP");
        CTREErrorCodeHelper.printError(
            this.wrappedObject.config_kI(slotId, i, TalonFXWrapper.timeoutMS),
            "TalonFX.setPIDF_kI");
        CTREErrorCodeHelper.printError(
            this.wrappedObject.config_kD(slotId, d, TalonFXWrapper.timeoutMS),
            "TalonFX.setPIDF_kD");
        CTREErrorCodeHelper.printError(
            this.wrappedObject.config_kF(slotId, f, TalonFXWrapper.timeoutMS),
            "TalonFX.setPIDF_kF");
    }

    public void setMotionMagicPIDF(double p, double i, double d, double f, double velocity, double acceleration, int slotId)
    {
        CTREErrorCodeHelper.printError(
            this.wrappedObject.config_kP(slotId, p, TalonFXWrapper.timeoutMS),
            "TalonFX.setMotionMagicPIDF_kP");
        CTREErrorCodeHelper.printError(
            this.wrappedObject.config_kI(slotId, i, TalonFXWrapper.timeoutMS),
            "TalonFX.setMotionMagicPIDF_kI");
        CTREErrorCodeHelper.printError(
            this.wrappedObject.config_kD(slotId, d, TalonFXWrapper.timeoutMS),
            "TalonFX.setMotionMagicPIDF_kD");
        CTREErrorCodeHelper.printError(
            this.wrappedObject.config_kF(slotId, f, TalonFXWrapper.timeoutMS),
            "TalonFX.setMotionMagicPIDF_kF");
        CTREErrorCodeHelper.printError(
            this.wrappedObject.configMotionCruiseVelocity(velocity, TalonFXWrapper.timeoutMS),
            "TalonFX.setMotionMagicPIDF_CruiseVelocity");
        CTREErrorCodeHelper.printError(
            this.wrappedObject.configMotionAcceleration(acceleration, TalonFXWrapper.timeoutMS),
            "TalonFX.setMotionMagicPIDF_Acceleration");
    }

    public void setPIDF(double p, double i, double d, double f, int izone, double closeLoopRampRate, int slotId)
    {
        CTREErrorCodeHelper.printError(
            this.wrappedObject.config_kP(slotId, p, TalonFXWrapper.timeoutMS),
            "TalonFX.setPIDF_kP");
        CTREErrorCodeHelper.printError(
            this.wrappedObject.config_kI(slotId, i, TalonFXWrapper.timeoutMS),
            "TalonFX.setPIDF_kI");
        CTREErrorCodeHelper.printError(
            this.wrappedObject.config_kD(slotId, d, TalonFXWrapper.timeoutMS),
            "TalonFX.setPIDF_kD");
        CTREErrorCodeHelper.printError(
            this.wrappedObject.config_kF(slotId, f, TalonFXWrapper.timeoutMS),
            "TalonFX.setPIDF_kF");
        CTREErrorCodeHelper.printError(
            this.wrappedObject.config_IntegralZone(slotId, izone, TalonFXWrapper.timeoutMS),
            "TalonFX.setPIDF_IntegralZone");
        CTREErrorCodeHelper.printError(
            this.wrappedObject.configClosedloopRamp(closeLoopRampRate, TalonFXWrapper.timeoutMS),
            "TalonFX.setPIDF_CloosedloopRamp");
    }

    public void setForwardLimitSwitch(boolean enabled, boolean normallyOpen)
    {
        LimitSwitchSource source = LimitSwitchSource.Deactivated;
        if (enabled)
        {
            source = LimitSwitchSource.FeedbackConnector;
        }

        LimitSwitchNormal type = LimitSwitchNormal.NormallyClosed;
        if (normallyOpen)
        {
            type = LimitSwitchNormal.NormallyOpen;
        }

        CTREErrorCodeHelper.printError(
            this.wrappedObject.configForwardLimitSwitchSource(
                source,
                type,
                TalonFXWrapper.timeoutMS),
            "TalonFX.setForwardLimitSwitch");
    }

    public void setReverseLimitSwitch(boolean enabled, boolean normallyOpen)
    {
        LimitSwitchSource source = LimitSwitchSource.Deactivated;
        if (enabled)
        {
            source = LimitSwitchSource.FeedbackConnector;
        }

        LimitSwitchNormal type = LimitSwitchNormal.NormallyClosed;
        if (normallyOpen)
        {
            type = LimitSwitchNormal.NormallyOpen;
        }

        CTREErrorCodeHelper.printError(
            this.wrappedObject.configReverseLimitSwitchSource(
                source,
                type,
                TalonFXWrapper.timeoutMS),
            "TalonFX.setReverseLimitSwitch");
    }

    public void setInvertOutput(boolean invert)
    {
        this.wrappedObject.setInverted(invert);
    }

    public void setInvertSensor(boolean invert)
    {
        this.wrappedObject.setSensorPhase(invert);
    }

    public void setInvert(TalonFXInvertType invertType)
    {
        com.ctre.phoenix.motorcontrol.TalonFXInvertType ctreInvertType;
        switch (invertType)
        {
            case CounterClockwise:
                ctreInvertType = com.ctre.phoenix.motorcontrol.TalonFXInvertType.CounterClockwise;
                break;

            case FollowMaster:
                ctreInvertType = com.ctre.phoenix.motorcontrol.TalonFXInvertType.FollowMaster;
                break;

            case OpposeMaster:
                ctreInvertType = com.ctre.phoenix.motorcontrol.TalonFXInvertType.OpposeMaster;
                break;

            default:
            case Clockwise:
                ctreInvertType = com.ctre.phoenix.motorcontrol.TalonFXInvertType.Clockwise;
                break;
        }

        this.wrappedObject.setInverted(ctreInvertType);
    }

    public void setNeutralMode(MotorNeutralMode neutralMode)
    {
        NeutralMode mode;
        if (neutralMode == MotorNeutralMode.Brake)
        {
            mode = NeutralMode.Brake;
        }
        else
        {
            mode = NeutralMode.Coast;
        }

        this.wrappedObject.setNeutralMode(mode);
    }

    public void setVoltageCompensation(boolean enabled, double maxVoltage)
    {
        CTREErrorCodeHelper.printError(
            this.wrappedObject.configVoltageCompSaturation(maxVoltage, TalonFXWrapper.timeoutMS),
            "TalonFX.setVoltageCompensationSaturation");
        this.wrappedObject.enableVoltageCompensation(enabled);
    }

    public void setSupplyCurrentLimit(boolean enabled, double currentLimit, double triggerThresholdCurrent, double triggerThresholdTime)
    {
        SupplyCurrentLimitConfiguration config = new SupplyCurrentLimitConfiguration(enabled, currentLimit, triggerThresholdCurrent, triggerThresholdTime);
        CTREErrorCodeHelper.printError(
            this.wrappedObject.configSupplyCurrentLimit(config),
            "TalonFX.setSupplyCurrentLimit");
    }

    public void stop()
    {
        this.wrappedObject.set(ControlMode.Disabled, 0.0);
    }

    public void setPosition(double position)
    {
        CTREErrorCodeHelper.printError(
            this.wrappedObject.setSelectedSensorPosition(position, TalonFXWrapper.pidIdx, TalonFXWrapper.timeoutMS),
            "TalonFX.setPosition");
    }

    public void reset()
    {
        CTREErrorCodeHelper.printError(
            this.wrappedObject.setSelectedSensorPosition(0.0, TalonFXWrapper.pidIdx, TalonFXWrapper.timeoutMS),
            "TalonFX.reset");
    }

    public double getPosition()
    {
        return this.wrappedObject.getSelectedSensorPosition(TalonFXWrapper.pidIdx);
    }

    public double getVelocity()
    {
        return this.wrappedObject.getSelectedSensorVelocity(TalonFXWrapper.pidIdx);
    }

    public double getError()
    {
        return this.wrappedObject.getClosedLoopError(TalonFXWrapper.pidIdx);
    }

    public TalonXLimitSwitchStatus getLimitSwitchStatus()
    {
        TalonFXSensorCollection collection = this.wrappedObject.getSensorCollection();

        return new TalonXLimitSwitchStatus(
            collection.isFwdLimitSwitchClosed() == 1,
            collection.isRevLimitSwitchClosed() == 1);
    }
}
