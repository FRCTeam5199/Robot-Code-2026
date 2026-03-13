package frc.robot.constants;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

public class IntakePivotConstants {
    public static final int MOTOR_ID = 14;
    public static final int ENCODER_ID = 16;

    public static final double STATOR_CURRENT_LIMIT = 60;
    public static final double SUPPLY_CURRENT_LIMIT = 60;
    public static final boolean INVERTED = false;
    public static final boolean BRAKE = true;

    public static final double DEPLOY = 101; //101 with bumper, 98 without
    public static final double UPAGITATE = 75;
    public static final double DOWNAGITATE = 90;
    public static final double STOW = 5;

    public static final Slot0Configs SLOT0_CONFIGS = new Slot0Configs()
            .withKP(60)
            .withKI(0)
            .withKD(1)
            .withKS(0.25)
            .withKG(0.25)
            .withKV(.15384615384615384615384615384615)
            .withKA(0)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign)
            .withGravityType(GravityTypeValue.Arm_Cosine)
            .withGravityArmPositionOffset(.0);

    public static final double VELOCITY = 5;
    public static final double ACCELERATION = 7;
    public static final double JERK = 70;

    public static final double LOWER_TOLERANCE = 4;
    public static final double UPPER_TOLERANCE = 4;

    public static final double MAGNET_OFFSET = .031494140625;
    public static final boolean ENCODER_DIRECTION = false;

    public static final double MIN = 0;
    public static final double MAX = 108;

    public static final double GEAR_RATIO = 20d;
    public static final double[][] GEAR_RATIOA = {{20, 1}};
    public static final double INTAKE_SENSOR_MECH_RATIO = 1d;
}
