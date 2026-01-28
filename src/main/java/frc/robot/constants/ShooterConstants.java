package frc.robot.constants;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

public class ShooterConstants {
    public static final int SHOOTER_MOTOR_ID = 0;
    public static final double SHOOTER_STATOR_CURRENT_LIMIT = 60;
    public static final double SHOOTER_SUPPLY_CURRENT_LIMIT = 60;
    public static final boolean SHOOTER_INVERTED = true;
    public static final boolean SHOOTER_BRAKE = true;

    public static final Slot0Configs SHOOTER_SLOT0_CONFIGS = new Slot0Configs()
            .withKP(0)
            .withKI(0)
            .withKD(0)
            .withKS(0)
            .withKG(0)
            .withKV(0)
            .withKA(0)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);

    public static final double SHOOTER_ACCELERATION = 400;
    public static final double SHOOTER_JERK = 4000;

    public static final double SHOOTER_LOWER_TOLERANCE = 2;
    public static final double SHOOTER_UPPER_TOLERANCE = 2;

    public static final double[][] SHOOTER_GEAR_RATIO = {{1, 1}};
}
