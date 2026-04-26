package frc.robot.constants;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

public class IntakePivotConstants {
    public static final int MOTOR_ID = 17;
    public static final int ENCODER_ID = 99;

    public static final double STATOR_CURRENT_LIMIT = 40;
    public static final double SUPPLY_CURRENT_LIMIT = 40;
    public static final boolean INVERTED = false;
    public static final boolean BRAKE = true;

    public static final double DEPLOY = 98;
    public static final double UPAGITATE = 70;
    public static final double DOWNAGITATE = 90;
    public static final double STOW = -1;

    public static final Slot0Configs SLOT0_CONFIGS = new Slot0Configs()
            .withKP(10) //10
            .withKI(0)
            .withKD(0)
            .withKS(0.3)
            .withKG(0.04)
            .withKV(.14858841010401188707280832095097)
            .withKA(0)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign)
            .withGravityType(GravityTypeValue.Arm_Cosine)
            .withGravityArmPositionOffset(.0);

    public static final double VELOCITY = 50; //125
    public static final double ACCELERATION = 100; //200
    public static final double JERK = 1500; //3000

    public static final double LOWER_TOLERANCE = 10;
    public static final double UPPER_TOLERANCE = 10;

    public static final double MAGNET_OFFSET = .34912109375;
    public static final boolean ENCODER_DIRECTION = true;

    public static final double MIN = -2;
    public static final double MAX = 85;

    //    public static final double GEAR_RATIO = 48;
    public static final double[][] GEAR_RATIOA = {{48, 1}};
    public static final double INTAKE_SENSOR_MECH_RATIO = 1d;

    public static final CANBus canbus = new CANBus("rio");
}
