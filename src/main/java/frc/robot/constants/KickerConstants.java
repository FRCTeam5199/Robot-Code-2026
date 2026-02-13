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

//    public static final Slot0Configs KICKER_SLOT0_CONFIGS = new Slot0Configs()
//            .withKP(99999999999999999999d)
//            .withKI(0)
//            .withKD(0)
//            .withKS(.26)
//            .withKG(0)
//            .withKV(1 / 4.9)
//            .withKA(0)
//            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);


    public static final Slot0Configs KICKER_SLOT0_CONFIGS = new Slot0Configs()
            .withKP(2)
            .withKI(0)
            .withKD(0)
            .withKS(.26)
            .withKG(0)
            .withKV((1 / 4.9))
            .withKA(0)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);

    public static final double KICKER_ACCELERATION = 50;
    public static final double KICKER_JERK = 500;

    public static final double KICKER_LOWER_TOLERANCE = 3;
    public static final double KICKER_UPPER_TOLERANCE = 3;

    public static final double[][] KICKER_GEAR_RATIO = {{1, 1}};
}
