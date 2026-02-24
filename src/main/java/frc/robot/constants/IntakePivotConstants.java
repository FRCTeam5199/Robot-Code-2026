package frc.robot.constants;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

public class IntakePivotConstants {
    public static final int INTAKE_PIVOT_MOTOR_ID = 14;
    public static final int INTAKE_PIVOT_ENCODER_ID = 16;

    public static final double INTAKE_PIVOT_STATOR_CURRENT_LIMIT = 60;
    public static final double INTAKE_PIVOT_SUPPLY_CURRENT_LIMIT = 60;
    public static final boolean INTAKE_PIVOT_INVERTED = false;
    public static final boolean INTAKE_PIVOT_BRAKE = true;

    public static final double INTAKE_OUT = 108;
    public static final double INTAKE_IN = 5;

    public static final Slot0Configs INTAKE_PIVOT_SLOT0_CONFIGS = new Slot0Configs()
            .withKP(25)
            .withKI(0)
            .withKD(0)
            .withKS(.17)
            .withKG(.25)
            .withKV(.09090909090909090909090909090909)
            .withKA(0)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign)
            .withGravityType(GravityTypeValue.Arm_Cosine)
            .withGravityArmPositionOffset(.055555555555555555); //figure out how to use this

    public static final double INTAKE_PIVOT_VELOCITY_OUT = 8;
    public static final double INTAKE_PIVOT_ACCELERATION_OUT = 16;
    public static final double INTAKE_PIVOT_JERK_OUT = 100;

    public static final double INTAKE_PIVOT_VELOCITY_IN = .5;
    public static final double INTAKE_PIVOT_ACCELERATION_IN = 4;
    public static final double INTAKE_PIVOT_JERK_IN = 10;

    public static final double INTAKE_PIVOT_LOWER_TOLERANCE = 4;
    public static final double INTAKE_PIVOT_UPPER_TOLERANCE = 4;

    public static final double INTAKE_PIVOT_MAGNET_OFFSET = -0.3239746;
    public static final boolean INTAKE_PIVOT_ENCODER_DIRECTION = false;

    public static final double INTAKE_PIVOT_MIN = 3;
    public static final double INTAKE_PIVOT_MAX = 109;

    public static final double INTAKE_PIVOT_GEAR_RATIO = 20d;
    public static final double[][] INTAKE_PIVOT_GEAR_RATIOA = {{20, 1}};
    public static final double INTAKE_SENSOR_MECH_RATIO = 1d;
}
