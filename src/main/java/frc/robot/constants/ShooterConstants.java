package frc.robot.constants;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

public class ShooterConstants {
    public static final int MOTOR_ID = 22;
    public static final int SECOND_MOTOR_ID = 20;
    public static final double STATOR_CURRENT_LIMIT = 100;
    public static final double SUPPLY_CURRENT_LIMIT = 60;
    public static final boolean INVERTED = true;
    public static final boolean SECOND_INVERTED = true;
    public static final boolean BRAKE = false;

    public static final Slot0Configs SLOT0_CONFIGS = new Slot0Configs()
            .withKP(.00096084)
            .withKI(0)
            .withKD(0)
            .withKS(.26163)
            .withKG(0)
            .withKV(.12255)
            .withKA(.0036266)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);

    public static final double ACCELERATION = 80000;
    public static final double JERK = 80000000;

    public static final double LOWER_TOLERANCE = 4;
    public static final double UPPER_TOLERANCE = 4;

    public static final double[][] GEAR_RATIO = {{1, 1}};

    public static final CANBus CANBUS = new CANBus("Shooter");
}
