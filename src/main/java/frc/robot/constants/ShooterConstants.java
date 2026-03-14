package frc.robot.constants;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

public class ShooterConstants {
    public static final int MOTOR_ID = 22;
    public static final int SECOND_MOTOR_ID = 26;
    public static final double STATOR_CURRENT_LIMIT = 60;
    public static final double SUPPLY_CURRENT_LIMIT = 60;
    public static final boolean INVERTED = false;
    public static final boolean SECOND_INVERTED = true;
    public static final boolean BRAKE = false;

    public static final Slot0Configs SLOT0_CONFIGS = new Slot0Configs()
            .withKP(0.25)
            .withKI(0)
            .withKD(0.01)
            .withKS(0.33)
            .withKG(0)
            .withKV(.12048192771084337349397590361446)
            .withKA(0)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);

    public static final double ACCELERATION = 400;
    public static final double JERK = 4000;

    public static final double LOWER_TOLERANCE = 1 + .5;
    public static final double UPPER_TOLERANCE = 1 + .5;

    public static final double[][] GEAR_RATIO = {{1, 1}};
}
