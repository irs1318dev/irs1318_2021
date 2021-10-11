package frc.robot.common;

public abstract class SettingDefaultValue
{
    private final SettingType type;
    private final Object defaultValue;
    protected <T> SettingDefaultValue(T defaultValue)
    {
        this.type = SettingDefaultValue.getSettingType(defaultValue);
        this.defaultValue = defaultValue;
    }

    public abstract ISettingKey getKey();

    public SettingType getSettingType()
    {
        return this.type;
    }

    public boolean getDefaultBool()
    {
        if (this.type != SettingType.Boolean)
        {
            throw new RuntimeException("Unexpected boolean retrieval for " + this.getKey().name() + " of type " + this.type);
        }

        return (boolean)this.defaultValue;
    }

    public int getDefaultInt()
    {
        if (this.type != SettingType.Integer)
        {
            throw new RuntimeException("Unexpected int retrieval for " + this.getKey().name() + " of type " + this.type);
        }

        return (int)this.defaultValue;
    }

    public long getDefaultLong()
    {
        if (this.type != SettingType.Long)
        {
            throw new RuntimeException("Unexpected long retrieval for " + this.getKey().name() + " of type " + this.type);
        }

        return (long)this.defaultValue;
    }

    public double getDefaultDouble()
    {
        if (this.type != SettingType.Double)
        {
            throw new RuntimeException("Unexpected double retrieval for " + this.getKey().name() + " of type " + this.type);
        }

        return (double)this.defaultValue;
    }

    public String getDefaultString()
    {
        if (this.type != SettingType.String)
        {
            throw new RuntimeException("Unexpected string retrieval for " + this.getKey().name() + " of type " + this.type);
        }

        return (String)this.defaultValue;
    }

    private static SettingType getSettingType(Object value)
    {
        String className = null;
        if (value != null)
        {
            Class valueClass = value.getClass();
            for (SettingType settingType : SettingType.values())
            {
                if (valueClass == settingType.getSettingClass())
                {
                    return settingType;
                }
            }

            className = valueClass.toString();
        }

        throw new RuntimeException("Unsupported type " + className);
    }
}