package frc.robot.constants;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

public class ShooterConstants {
    public static final int SHOOTER_MOTOR_ID = 22;
    public static final int SECOND_SHOOTER_MOTOR_ID = 26;
    public static final double SHOOTER_STATOR_CURRENT_LIMIT = 60;
    public static final double SHOOTER_SUPPLY_CURRENT_LIMIT = 60;
    public static final boolean SHOOTER_INVERTED = false;
    public static final boolean SECOND_SHOOTER_INVERTED = true;
    public static final boolean SHOOTER_BRAKE = false;

    public static final Slot0Configs SHOOTER_SLOT0_CONFIGS = new Slot0Configs()
            .withKP(.5)
            .withKI(0)
            .withKD(0.001)
            .withKS(.25)
            .withKG(0)
            .withKV(.1242236025)
            .withKA(0)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);

    public static final double SHOOTER_ACCELERATION = 400;
    public static final double SHOOTER_JERK = 4000;

    public static final double SHOOTER_LOWER_TOLERANCE = 3;
    public static final double SHOOTER_UPPER_TOLERANCE = 3;

    public static final double[][] SHOOTER_GEAR_RATIO = {{1, 1}};
}
