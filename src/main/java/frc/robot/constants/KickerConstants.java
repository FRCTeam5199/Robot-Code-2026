package frc.robot.constants;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

public class KickerConstants {
    public static final int KICKER_MOTOR_ID = 15;
    public static final double KICKER_STATOR_CURRENT_LIMIT = 60;
    public static final double KICKER_SUPPLY_CURRENT_LIMIT = 60;
    public static final boolean KICKER_INVERTED = true;
    public static final boolean KICKER_BRAKE = true;

    public static final Slot0Configs KICKER_SLOT0_CONFIGS = new Slot0Configs()
            .withKP(0)
            .withKI(0)
            .withKD(0)
            .withKS(0)
            .withKG(0)
            .withKV(0)
            .withKA(0)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);

    public static final double KICKER_ACCELERATION = 400;
    public static final double KICKER_JERK = 4000;

    public static final double KICKER_LOWER_TOLERANCE = 2;
    public static final double KICKER_UPPER_TOLERANCE = 2;

    public static final double[][] KICKER_GEAR_RATIO = {{1, 1}};
}
