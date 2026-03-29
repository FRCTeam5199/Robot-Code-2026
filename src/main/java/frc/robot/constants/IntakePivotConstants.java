package frc.robot.constants;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

public class IntakePivotConstants {
    public static final int MOTOR_ID = 17;
    public static final int ENCODER_ID = 99;

    public static final double STATOR_CURRENT_LIMIT = 60;
    public static final double SUPPLY_CURRENT_LIMIT = 60;
    public static final boolean INVERTED = false;
    public static final boolean BRAKE = true;

    public static final double DEPLOY = 206; //101 with bumper, 98 without
    public static final double UPAGITATE = 70;
    public static final double DOWNAGITATE = 90;
    public static final double STOW = 5;

    public static final Slot0Configs SLOT0_CONFIGS = new Slot0Configs()
            .withKP(60)
            .withKI(0)
            .withKD(1)
            .withKS(0.37)
            .withKG(0.3)
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

    public static final double MAGNET_OFFSET = .34912109375;
    public static final boolean ENCODER_DIRECTION = true;

    public static final double MIN = -1;
    public static final double MAX = 206;

    public static final double GEAR_RATIO = 20d;
    public static final double[][] GEAR_RATIOA = {{20, 1}};
    public static final double INTAKE_SENSOR_MECH_RATIO = 1d;

    public static final CANBus canbus = new CANBus("rio");
}
