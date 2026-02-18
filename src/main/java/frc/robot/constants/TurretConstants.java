package frc.robot.constants;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

public class TurretConstants {
    public static final int TURRET_MOTOR_ID = 20;
    public static final int TURRET_ENCODER_ID = 23;

    public static final double TURRET_STATOR_CURRENT_LIMIT = 60;
    public static final double TURRET_SUPPLY_CURRENT_LIMIT = 60;

    public static final boolean TURRET_INVERTED = false;
    public static final boolean TURRET_BRAKE = true;

    public static final Slot0Configs TURRET_SLOT0_CONFIGS = new Slot0Configs()
            .withKP(0)
            .withKI(0)
            .withKD(0)
            .withKS(0)
            .withKG(0)
            .withKV(.10067114093959731543624161073826)
            .withKA(0)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);

    public static final double TURRET_VELOCITY = 150;
    public static final double TURRET_ACCELERATION = 400;
    public static final double TURRET_JERK = 4000;

    public static final double TURRET_LOWER_TOLERANCE = 2;
    public static final double TURRET_UPPER_TOLERANCE = 2;

    public static final double[][] TURRET_GEAR_RATIO = {{50, 12}, {100, 10}};

    public static final double TURRET_ENCODER_MAGNET_OFFSET = 0.177734375;
    public static final double TURRET_SENSOR_TO_MECH_GEAR_RATIO = 100d / 10d;
    public static final double TURRET_MOTOR_TO_SENSOR_GEAR_RATIO = 50d / 12d;
    public static final boolean TURRET_CCW_POSITIVE = true;
    public static final double TURRET_ABSOLUTE_DISCONTINUITY_POINT = .5;

    public static final double TURRET_MIN = -185d;
    public static final double TURRET_MAX = 185d;
}
