package frc.robot.common.robotprovider;

public interface IPigeon2
{
    /**
     * Get Yaw, Pitch, and Roll data.
     * @param ypr_deg array to fill with yaw[0], pitch[1], and roll[2] data. Yaw is within [-368,640, +368,640] degrees. Pitch is within [-90,+90] degrees. Roll is within [-90,+90] degrees.
     */
    void getYawPitchRoll(double[] ypr_deg);

    /**
     * Sets the Yaw register to the specified value.
     * @param angleDeg New yaw in degrees [+/- 368,640 degrees]
     */
    void setYaw(double angleDeg);
}
