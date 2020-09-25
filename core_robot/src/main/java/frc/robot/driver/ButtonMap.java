package frc.robot.driver;

import javax.inject.Singleton;

import frc.robot.*;
import frc.robot.common.Helpers;
import frc.robot.driver.common.*;
import frc.robot.driver.common.buttons.*;
import frc.robot.driver.common.descriptions.*;
import frc.robot.driver.controltasks.*;

@Singleton
public class ButtonMap implements IButtonMap
{
    private static ShiftDescription[] ShiftButtonSchema = new ShiftDescription[]
    {
        new ShiftDescription(
            Shift.DriverDebug,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_LEFT_BUTTON),
        new ShiftDescription(
            Shift.OperatorDebug,
            UserInputDevice.Operator,
            UserInputDeviceButton.PS4_LEFT_BUTTON),
    };

    public static AnalogOperationDescription[] AnalogOperationSchema = new AnalogOperationDescription[]
    {
        new AnalogOperationDescription(
            AnalogOperation.DriveTrainMoveForward,
            UserInputDevice.Driver,
            AnalogAxis.XBONE_LSY,
            ElectronicsConstants.INVERT_XBONE_LEFT_Y_AXIS,
            TuningConstants.DRIVETRAIN_DEAD_ZONE
        ),
        new AnalogOperationDescription(
            AnalogOperation.DriveTrainMoveSide,
            UserInputDevice.Driver,
            AnalogAxis.XBONE_LSX,
            ElectronicsConstants.INVERT_XBONE_LEFT_X_AXIS,
            TuningConstants.DRIVETRAIN_DEAD_ZONE
        ),
        new AnalogOperationDescription(
            AnalogOperation.DriveTrainTurnX,
            UserInputDevice.Driver,
            AnalogAxis.XBONE_RSX,
            ElectronicsConstants.INVERT_XBONE_RIGHT_X_AXIS,
            TuningConstants.DRIVETRAIN_DEAD_ZONE
        ),
        new AnalogOperationDescription(
            AnalogOperation.DriveTrainTurnY,
            UserInputDevice.Driver,
            AnalogAxis.XBONE_RSY,
            ElectronicsConstants.INVERT_XBONE_RIGHT_Y_AXIS,
            TuningConstants.DRIVETRAIN_DEAD_ZONE
        ),

    };

    public static DigitalOperationDescription[] DigitalOperationSchema = new DigitalOperationDescription[]
    {
        new DigitalOperationDescription(
            DigitalOperation.DriveTrainReset,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_A_BUTTON,
            Shift.DriverDebug,
            Shift.None,
            ButtonType.Click
        )
    };

    public static MacroOperationDescription[] MacroSchema = new MacroOperationDescription[]
    {
        // DriveTrain macros
    };

    @Override
    public ShiftDescription[] getShiftSchema()
    {
        return ButtonMap.ShiftButtonSchema;
    }

    @Override
    public AnalogOperationDescription[] getAnalogOperationSchema()
    {
        return ButtonMap.AnalogOperationSchema;
    }

    @Override
    public DigitalOperationDescription[] getDigitalOperationSchema()
    {
        return ButtonMap.DigitalOperationSchema;
    }

    @Override
    public MacroOperationDescription[] getMacroOperationSchema()
    {
        return ButtonMap.MacroSchema;
    }
}
