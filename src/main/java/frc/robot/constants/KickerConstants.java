package frc.robot.constants;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

public class KickerConstants {
    public static final int KICKER_UPPER_MOTOR_ID = 18;
    public static final int KICKER_LOWER_MOTOR_ID = 19;
    public static final double KICKER_STATOR_CURRENT_LIMIT = 60;
    public static final double KICKER_SUPPLY_CURRENT_LIMIT = 60;
    public static final boolean KICKER_LOWER_INVERTED = true;
    public static final boolean KICKER_UPPER_INVERTED = false;
    public static final boolean KICKER_BRAKE = false;

    public static final Slot1Configs KICKER_SLOT1_CONFIGS = new Slot1Configs()
            .withKP(0)
            .withKI(0)
            .withKD(0)
            .withKS(.26)
            .withKG(0)
            .withKV(1 / 4.9)
            .withKA(0)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);


    public static final Slot0Configs KICKER_SLOT0_CONFIGS = new Slot0Configs()
            .withKP(1.5)
            .withKI(0)
            .withKD(0.005)
            .withKS(.34)
            .withKG(0)
            .withKV(.12195121951219512195121951219512)
            .withKA(0)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);

    public static final double KICKER_ACCELERATION = 200;
    public static final double KICKER_JERK = 2000;

    public static final double KICKER_LOWER_TOLERANCE = 20;
    public static final double KICKER_UPPER_TOLERANCE = 20;

    public static final double[][] KICKER_GEAR_RATIO = {{1, 1}};
}
