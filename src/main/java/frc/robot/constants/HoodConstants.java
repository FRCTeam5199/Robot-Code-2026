package frc.robot.constants;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

public class HoodConstants {
    public static final int HOOD_MOTOR_ID = 0;
    public static final int HOOD_ENCODER_ID = 0;

    public static final double HOOD_STATOR_CURRENT_LIMIT = 60;
    public static final double HOOD_SUPPLY_CURRENT_LIMIT = 60;
    public static final boolean HOOD_INVERTED = true;
    public static final boolean HOOD_BRAKE = true;

    public static final Slot0Configs HOOD_SLOT0_CONFIGS = new Slot0Configs()
            .withKP(5)
            .withKI(0)
            .withKD(0)
            .withKS(0.28)
            .withKG(0)
            .withKV(.08064516129032258064516129032258)
            .withKA(0)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign)
            .withGravityType(GravityTypeValue.Arm_Cosine)
            .withGravityArmPositionOffset(0);

    public static final double HOOD_VELOCITY = 150;
    public static final double HOOD_ACCELERATION = 800;
    public static final double HOOD_JERK = 8000;

    public static final double HOOD_LOWER_TOLERANCE = 2;
    public static final double HOOD_UPPER_TOLERANCE = 2;

    public static final double HOOD_MAGNET_OFFSET = -0.377197265625;// -0.58203125;
    public static final boolean HOOD_IS_CCW_POS = false;

    public static final double HOOD_MIN = 2; //2
    public static final double HOOD_MAX = 29;  //29

    public static final double HOOD_MOTOR_TO_SENSOR_GEAR_RATIO = 8d;
    public static final double[][] HOOD_GEAR_RATIO = {{48, 12}, {30, 15}, {180, 10}};
    public static final double HOOD_SENSOR_TO_MECH_GEAR_RATIO = 18d;

    public static final double HOOD_ABSOLUTE_DISCONTINUITY_POINT = 1;
}


