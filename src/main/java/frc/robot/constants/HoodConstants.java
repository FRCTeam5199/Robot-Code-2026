package frc.robot.constants;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

public class HoodConstants {
    public static final int HOOD_MOTOR_ID = 0;
    public static final int HOOD_ENCODER_ID = 0;

    public static final double HOOD_STATOR_CURRENT_LIMIT = 60;
    public static final double HOOD_SUPPLY_CURRENT_LIMIT = 60;
    public static final boolean HOOD_INVERTED = false;
    public static final boolean HOOD_BRAKE = true;

    public static final Slot0Configs HOOD_SLOT0_CONFIGS = new Slot0Configs()
            .withKP(0)
            .withKI(0)
            .withKD(0)
            .withKS(0)
            .withKG(0)
            .withKV(0)
            .withKA(0)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign)
            .withGravityType(GravityTypeValue.Arm_Cosine)
            .withGravityArmPositionOffset(0); //figure out how to use this

    public static final double HOOD_VELOCITY = 0;
    public static final double HOOD_ACCELERATION = 0;
    public static final double HOOD_JERK = 0;

    public static final double HOOD_LOWER_TOLERANCE = 0;
    public static final double HOOD_UPPER_TOLERANCE = 0;

    public static final double HOOD_MAGNET_OFFSET = 0;
    public static final boolean HOOD_ENCODER_DIRECTION = true;

    public static final double HOOD_MIN = 0;
    public static final double HOOD_MAX = 0;

    public static final double HOOD_GEAR_RATIO = 0;
    public static final double[][] HOOD_GEAR_RATIOA = {{0, 0}};
    public static final double HOOD_SENSOR_MECH_RATIO = 0;
}


