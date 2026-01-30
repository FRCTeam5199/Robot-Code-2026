package frc.robot.constants;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

public class TurretConstants {
    public static final int TURRET_MOTOR_ID = 19;
    public static final int TURRET_ENCODER_ID = 20;

    public static final double TURRET_STATOR_CURRENT_LIMIT = 60;
    public static final double TURRET_SUPPLY_CURRENT_LIMIT = 60;

    public static final boolean TURRET_INVERTED = true;
    public static final boolean TURRET_BRAKE = true;

    public static final Slot0Configs TURRET_SLOT0_CONFIGS = new Slot0Configs()
            .withKP(0)
            .withKI(0)
            .withKD(0)
            .withKS(0)
            .withKG(0)
            .withKV(0)
            .withKA(0)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);

    public static final double TURRET_ACCELERATION = 400;
    public static final double TURRET_JERK = 4000;

    public static final double TURRET_LOWER_TOLERANCE = 2;
    public static final double TURRET_UPPER_TOLERANCE = 2;

    public static final double[][] TURRET_GEAR_RATIO = {{12, 50}, {102, 10}};

    public static final double TURRET_ENCODER_MAGNET_OFFSET = 0.177734375;
    public static final double TURRET_SENSOR_TO_MECH_GEAR_RATIO = .0980392156862745; //102:10
    public static final double TURRET_MOTOR_TO_SENSOR_GEAR_RATIO = 4.16667; //12 to 50
    public static final boolean TURRET_CCW_POSITIVE = true;

    public static final double TURRET_MIN = -185d;
    public static final double TURRET_MAX = 185d;
}
