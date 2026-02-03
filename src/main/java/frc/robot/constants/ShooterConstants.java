package frc.robot.constants;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

public class ShooterConstants {
    public static final int SHOOTER_MOTOR_ID = 21;
    public static final double SHOOTER_STATOR_CURRENT_LIMIT = 60;
    public static final double SHOOTER_SUPPLY_CURRENT_LIMIT = 60;
    public static final boolean SHOOTER_INVERTED = false;
    public static final boolean SHOOTER_BRAKE = false;

    public static final Slot0Configs SHOOTER_SLOT0_CONFIGS = new Slot0Configs()
            .withKP(1.5)
            .withKI(0)
            .withKD(0)
            .withKS(.37)
            .withKG(0)
            .withKV(.12195121951219512195121951219512)
            .withKA(0)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);

    public static final double SHOOTER_ACCELERATION = 400;
    public static final double SHOOTER_JERK = 4000;

    public static final double SHOOTER_LOWER_TOLERANCE = 4;
    public static final double SHOOTER_UPPER_TOLERANCE = 4;

    public static final double[][] SHOOTER_GEAR_RATIO = {{1, 1}};
}
