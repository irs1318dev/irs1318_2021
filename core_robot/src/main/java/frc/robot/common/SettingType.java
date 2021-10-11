package frc.robot.common;

public enum SettingType
{
    Boolean(Boolean.class),
    Integer(Integer.class),
    Long(Long.class),
    Double(Double.class),
    String(String.class);

    private final Class settingClass;
    private SettingType(Class settingClass)
    {
        this.settingClass = settingClass;
    }

    public Class getSettingClass()
    {
        return this.settingClass;
    }
}
