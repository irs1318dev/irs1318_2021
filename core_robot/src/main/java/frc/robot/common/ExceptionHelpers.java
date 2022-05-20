package frc.robot.common;

public class ExceptionHelpers
{
    public static String exceptionString(Exception ex)
    {
        StringBuilder b = new StringBuilder();
        b.append(ex.toString());
        b.append("\r\n");
        for (StackTraceElement stackTraceElement : ex.getStackTrace())
        {
            b.append(stackTraceElement.toString());
            b.append("\r\n");
        }

        return b.toString();
    }
}
