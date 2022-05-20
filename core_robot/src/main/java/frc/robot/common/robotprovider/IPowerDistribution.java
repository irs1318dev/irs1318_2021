package frc.robot.common.robotprovider;

public interface IPowerDistribution
{
    double getBatteryVoltage();
    double getCurrent(int pdpChannel);
    void setSwitchableChannel(boolean enabled);
}
