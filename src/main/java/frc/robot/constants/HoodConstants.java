package frc.robot.constants;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

public class HoodConstants {
    public static final int MOTOR_ID = 21;
    public static final int ENCODER_ID = 27;

    public static final double STATOR_CURRENT_LIMIT = 60;
    public static final double SUPPLY_CURRENT_LIMIT = 60;
    public static final boolean INVERTED = true;
    public static final boolean BRAKE = true;

    public static final Slot0Configs SLOT0_CONFIGS = new Slot0Configs()
            .withKP(5)
            .withKI(0)
            .withKD(0)
            .withKS(0.26)
            .withKG(0)
            .withKV(.09025270758122743682310469314079)
            .withKA(0)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign)
            .withGravityType(GravityTypeValue.Arm_Cosine)
            .withGravityArmPositionOffset(0);

    public static final double VELOCITY = 150;
    public static final double ACCELERATION = 400;
    public static final double JERK = 4000;

    public static final double LOWER_TOLERANCE = .5;
    public static final double UPPER_TOLERANCE = .5;

    public static final double MAGNET_OFFSET = .260986328125;
    public static final boolean IS_CCW_POS = false;

    public static final double MIN = 0;
    public static final double MAX = 29;

    public static final double MOTOR_TO_SENSOR_GEAR_RATIO = 8d;
    public static final double[][] GEAR_RATIO = {{48, 12}, {30, 15}, {180, 10}};
    public static final double SENSOR_TO_MECH_GEAR_RATIO = 18d;

    public static final double ABSOLUTE_DISCONTINUITY_POINT = .5;
}


