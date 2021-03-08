package frc.robot.mechanisms;

import frc.robot.TestProvider;
import frc.robot.common.robotprovider.INavx;
import frc.robot.common.robotprovider.ITalonFX;
import frc.robot.common.robotprovider.ITalonSRX;
import frc.robot.common.robotprovider.IVictorSPX;
import frc.robot.common.robotprovider.MotorNeutralMode;
import frc.robot.common.robotprovider.NullLogger;
import frc.robot.common.robotprovider.TalonSRXControlMode;
import frc.robot.common.robotprovider.TalonXFeedbackDevice;
import frc.robot.common.robotprovider.TalonXLimitSwitchStatus;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

public class DriveTrainMechanismTests
{
    @Test
    public void test1()
    {
        TestProvider provider = new TestProvider();
        MockNavx navx = new MockNavx();
        provider.setNavx(navx);

        MockTalonFX[] steer = new MockTalonFX[4];
        MockTalonFX[] drive = new MockTalonFX[4];
        for (int i = 0; i < 4; i++)
        {
            steer[i] = new MockTalonFX(2 * i + 1);
            drive[i] = new MockTalonFX(2 * i + 2);

            provider.setTalonFX(2 * i + 1, steer[i]);
            provider.setTalonFX(2 * i + 2, drive[i]);
        }
    }

    private class MockNavx implements INavx
    {
        private double currentAngle;

        MockNavx()
        {
        }

        @Override
        public boolean isConnected()
        {
            return true;
        }

        @Override
        public double getAngle()
        {
            return this.currentAngle;
        }

        @Override
        public double getDisplacementX()
        {
            return 0;
        }

        @Override
        public double getDisplacementY()
        {
            return 0;
        }

        @Override
        public double getDisplacementZ()
        {
            return 0;
        }

        @Override
        public void reset()
        {
            this.currentAngle = 0.0;
        }

        @Override
        public void resetDisplacement()
        {
        }

        public void set(double angle)
        {
            this.currentAngle = angle;
        }
    }

    private class MockTalonFX implements ITalonFX
    {
        private final int deviceId;
        private double currentValue;

        MockTalonFX(int deviceId)
        {
            this.deviceId = deviceId;
        }

        @Override
        public void follow(ITalonSRX talonSRX)
        {
        }

        @Override
        public void follow(ITalonFX talonFX)
        {
        }

        @Override
        public void follow(IVictorSPX victorSPX)
        {
        }

        @Override
        public void setControlMode(TalonSRXControlMode mode)
        {
        }

        @Override
        public void setSensorType(TalonXFeedbackDevice feedbackDevice)
        {
        }

        @Override
        public void setFeedbackFramePeriod(int periodMS)
        {
        }

        @Override
        public void setPIDFFramePeriod(int periodMS)
        {
        }

        @Override
        public void configureVelocityMeasurements(int periodMS, int windowSize)
        {
        }

        @Override
        public void configureAllowableClosedloopError(int slotId, int error)
        {
        }

        @Override
        public void setSelectedSlot(int slotId)
        {
        }

        @Override
        public void setPIDF(double p, double i, double d, double f, int slotId)
        {
        }

        @Override
        public void setMotionMagicPIDF(double p, double i, double d, double f, int velocity, int acceleration, int slotId)
        {
        }

        @Override
        public void setPIDF(double p, double i, double d, double f, int izone, double closeLoopRampRate, int slotId)
        {
        }

        @Override
        public void setForwardLimitSwitch(boolean enabled, boolean normallyOpen)
        {
        }

        @Override
        public void setReverseLimitSwitch(boolean enabled, boolean normallyOpen)
        {
        }

        @Override
        public void setInvertOutput(boolean flip)
        {
        }

        @Override
        public void setInvertSensor(boolean flip)
        {
        }

        @Override
        public void setNeutralMode(MotorNeutralMode neutralMode)
        {
        }

        @Override
        public void setVoltageCompensation(boolean enabled, double maxVoltage)
        {
        }

        @Override
        public void stop()
        {
        }

        @Override
        public void setPosition(double position)
        {
        }

        @Override
        public void reset()
        {
        }

        @Override
        public double getPosition() 
        {
            return this.currentValue;
        }

        @Override
        public double getVelocity()
        {
            return this.currentValue;
        }

        @Override
        public double getError()
        {
            return 0;
        }

        @Override
        public TalonXLimitSwitchStatus getLimitSwitchStatus()
        {
            return null;
        }

        @Override
        public void set(double power)
        {
            this.currentValue = power;
        }

        @Override
        public void setSupplyCurrentLimit(boolean enabled, double currentLimit, double triggerThresholdCurrent, double triggerThresholdTime)
        {
        }
    }
}
