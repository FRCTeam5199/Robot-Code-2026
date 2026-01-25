package frc.robot.generated;

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
    public static final double INTAKE_IN = 2;

    public static final Slot0Configs INTAKE_PIVOT_SLOT0_CONFIGS = new Slot0Configs()
            .withKP(22)
            .withKI(0)
            .withKD(0)
            .withKS(0.19)
            .withKG(0.29)
            .withKV(0.07692307692307692307692307692308)
            .withKA(0)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign)
            .withGravityType(GravityTypeValue.Arm_Cosine)
            .withGravityArmPositionOffset(0); //figure out how to use this

    public static final double INTAKE_PIVOT_VELOCITY_OUT = 30;
    public static final double INTAKE_PIVOT_ACCELERATION_OUT = 400;
    public static final double INTAKE_PIVOT_JERK_OUT = 1250;

    public static final double INTAKE_PIVOT_VELOCITY_IN = 18;
    public static final double INTAKE_PIVOT_ACCELERATION_IN = 350;
    public static final double INTAKE_PIVOT_JERK_IN = 1100;

    public static final double INTAKE_PIVOT_LOWER_TOLERANCE = 2;
    public static final double INTAKE_PIVOT_UPPER_TOLERANCE = 2;

    public static final double INTAKE_PIVOT_MAGNET_OFFSET = -0.3239746;
    public static final boolean INTAKE_PIVOT_ENCODER_DIRECTION = false;

    public static final double INTAKE_PIVOT_MIN = 1;
    public static final double INTAKE_PIVOT_MAX = 92;

    public static final double INTAKE_PIVOT_GEAR_RATIO = 20d;
    public static final double[][] INTAKE_PIVOT_GEAR_RATIOA = {{20,1}};
    public static final double INTAKE_SENSOR_MECH_RATIO = 1d;
}
