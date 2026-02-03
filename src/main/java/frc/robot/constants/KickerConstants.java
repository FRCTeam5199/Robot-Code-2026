package frc.robot.constants;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

public class KickerConstants {
    public static final int KICKER_MOTOR_ID = 18;
    public static final double KICKER_STATOR_CURRENT_LIMIT = 60;
    public static final double KICKER_SUPPLY_CURRENT_LIMIT = 60;
    public static final boolean KICKER_INVERTED = true;
    public static final boolean KICKER_BRAKE = false;

    public static final Slot0Configs KICKER_SLOT0_CONFIGS = new Slot0Configs()
            .withKP(15)
            .withKI(0)
            .withKD(0)
            .withKS(.29)
            .withKG(0)
            .withKV(.2570694087403599)
            .withKA(0)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);

    public static final double KICKER_ACCELERATION = 800;
    public static final double KICKER_JERK = 8000;

    public static final double KICKER_LOWER_TOLERANCE = 5;
    public static final double KICKER_UPPER_TOLERANCE = 5;

    public static final double[][] KICKER_GEAR_RATIO = {{1, 1}};
}
