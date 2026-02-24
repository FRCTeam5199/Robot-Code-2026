package frc.robot.constants;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

public class TurretConstants {
    public static final int MOTOR_ID = 20;
    public static final int ENCODER_ID = 23;

    public static final double STATOR_CURRENT_LIMIT = 60;
    public static final double SUPPLY_CURRENT_LIMIT = 60;

    public static final boolean INVERTED = false;
    public static final boolean BRAKE = true;

    public static final Slot0Configs SLOT0_CONFIGS = new Slot0Configs()
            .withKP(10)
            .withKI(0)
            .withKD(0)
            .withKS(.46)
            .withKG(0)
            .withKV(.09638554216867469879518072289157)
            .withKA(0)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);

    public static final double VELOCITY = 100;
    public static final double ACCELERATION = 300;
    public static final double JERK = 3000;

    public static final double LOWER_TOLERANCE = 1;
    public static final double UPPER_TOLERANCE = 1;

    public static final double[][] GEAR_RATIO = {{50, 12}, {100, 10}};

    public static final double ENCODER_MAGNET_OFFSET = -.122314453125;
    public static final double SENSOR_TO_MECH_GEAR_RATIO = 100d / 10d;
    public static final double MOTOR_TO_SENSOR_GEAR_RATIO = 50d / 12d;
    public static final boolean CCW_POSITIVE = false;
    public static final double ABSOLUTE_DISCONTINUITY_POINT = .5;

    public static final double MIN = -120d;
    public static final double MAX = 240d;
}