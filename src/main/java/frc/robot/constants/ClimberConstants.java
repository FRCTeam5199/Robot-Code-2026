package frc.robot.constants;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

public class ClimberConstants {
    public static final int MOTOR_ID = 99;//placeholder

    public static final double STATOR_CURRENT_LIMIT = 60;
    public static final double SUPPLY_CURRENT_LIMIT = 60;
    public static final boolean INVERTED = true;
    public static final boolean BRAKE = true;

    public static final Slot0Configs SLOT0_CONFIGS = new Slot0Configs()
            .withKP(1)
            .withKI(0)
            .withKD(0)
            .withKS(.19)
            .withKG(0)
            .withKV(1.59)
            .withKA(0)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign)
            .withGravityType(GravityTypeValue.Arm_Cosine)
            .withGravityArmPositionOffset(.0);

    public static final double VELOCITY = 5;
    public static final double ACCELERATION = 7;
    public static final double JERK = 70;

    public static final double LOWER_TOLERANCE = 4;
    public static final double UPPER_TOLERANCE = 4;

    public static final double MIN = 0;
    public static final double MAX = 94;

    public static final double[][] GEAR_RATIO = {{1, 50}};
    public static final double DRUM_CIRCUMFERENCE = Math.PI;
}
