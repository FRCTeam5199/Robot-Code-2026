package frc.robot.constants;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

public class KickerConstants {
    public static final int UPPER_MOTOR_ID = 18;
    public static final int LOWER_MOTOR_ID = 19;
    public static final double STATOR_CURRENT_LIMIT = 60;
    public static final double SUPPLY_CURRENT_LIMIT = 60;
    public static final boolean LOWER_INVERTED = false;
    public static final boolean UPPER_INVERTED = true;
    public static final boolean UPPER_BRAKE = false;

    public static final Slot0Configs UPPER_SLOT0_CONFIGS = new Slot0Configs()
            .withKP(0.5)
            .withKI(0)
            .withKD(0.01)
            .withKS(.36)
            .withKG(0)
            .withKV(.13043478260869565217391304347826)
            .withKA(0)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);


    public static final double ACCELERATION = 400;
    public static final double JERK = 4000;

    public static final double LOWER_TOLERANCE = 5d + 3d;
    public static final double UPPER_TOLERANCE = 5d + 3d;

    public static final double[][] GEAR_RATIO = {{1, 1}};
    public static final double SCALE_FACTOR = (1d);  
}
