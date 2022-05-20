package frc.robot.common.robotprovider;

public interface IJoystick
{
    boolean isConnected();

    double getAxis(int relevantAxis);

    int getPOV();

    boolean getRawButton(int value);
}
