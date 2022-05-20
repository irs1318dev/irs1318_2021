package frc.robot.common.robotprovider;

import com.ctre.phoenix.ErrorCode;

public class CTREErrorCodeHelper
{
    public static void printError(ErrorCode ec, String operation)
    {
        if (ec != ErrorCode.OK)
        {
            System.err.println(operation + " failed with " + ec.toString());
        }
    }
}
