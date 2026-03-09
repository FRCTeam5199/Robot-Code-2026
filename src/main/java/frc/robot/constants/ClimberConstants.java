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

    public static final double DEPLOY = .22;
    public static final double ZERO = 0;
    public static final double CLIMB = 0;

    public static final Slot0Configs SLOT0_CONFIGS = new Slot0Configs()
            .withKP(15)
            .withKI(0)
            .withKD(0)
            .withKS(.25)
            .withKG(0)
            .withKV(0.09551098376313276026743075453677)
            .withKA(0)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign)
            .withGravityType(GravityTypeValue.Arm_Cosine)
            .withGravityArmPositionOffset(.0);

    public static final double VELOCITY = 100;
    public static final double ACCELERATION = 400;
    public static final double JERK = 4000;

    public static final double LOWER_TOLERANCE = .02;
    public static final double UPPER_TOLERANCE = .02;

    public static final double MIN = 0;
    public static final double MAX = .25;

    public static final double[][] GEAR_RATIO = {{50, 1}};
    public static final double DRUM_CIRCUMFERENCE = Math.PI * .0254;

    public static final double CLIMB_TIME = 4;
}
