package frc.robot.constants;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

public class ClimberConstants {
    public static final int MOTOR_ID = 28;//placeholder

    public static final double STATOR_CURRENT_LIMIT = 60;
    public static final double SUPPLY_CURRENT_LIMIT = 60;
    public static final boolean INVERTED = true;
    public static final boolean BRAKE = true;

    public static final Slot0Configs SLOT0_CONFIGS = new Slot0Configs()
            .withKP(15)
            .withKI(0)
            .withKD(0)
            .withKS(.2)
            .withKG(0)
            .withKV(.10309278350515)
            .withKA(0)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign)
            .withGravityType(GravityTypeValue.Arm_Cosine)
            .withGravityArmPositionOffset(.0);

    public static final double VELOCITY = 5;
    public static final double ACCELERATION = 30;
    public static final double JERK = 300;

    public static final double LOWER_TOLERANCE = 4;
    public static final double UPPER_TOLERANCE = 4;

    public static final double MIN = 0;
    public static final double MAX = 94;

    public static final double[][] GEAR_RATIO = {{1, 50}};
    public static final double DRUM_CIRCUMFERENCE = Math.PI;
}
