package frc.robot.constants;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

public class IntakeLinearConstants {
    public static final int MOTOR_ID = 14;
    public static final int FOLLOWER_MOTOR_ID = 15;

    public static final double STATOR_CURRENT_LIMIT = 40;
    public static final double SUPPLY_CURRENT_LIMIT = 40;
    public static final boolean INVERTED = false;
    public static final boolean FOLLOWER_INVERTED = true;
    public static final boolean BRAKE = true;

    public static final double DEPLOY = 24.05;
    public static final double STOW = 7.6;

    public static final Slot0Configs SLOT0_CONFIGS = new Slot0Configs()
            .withKP(4)
            .withKI(0)
            .withKD(0)
            .withKS(0.07)
            .withKG(.44)
            .withKV(.125)
            .withKA(0)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);

    public static final double VELOCITY = 75; //125
    public static final double ACCELERATION = 110; //200
    public static final double JERK = 1500; //3000

    public static final double LOWER_TOLERANCE = 1;
    public static final double UPPER_TOLERANCE = 1;

    public static final double MIN = -0.2;
    public static final double MAX = 24.05; //24.05

    public static final double[][] GEAR_RATIOA = {{5, 1}};

    public static final CANBus CAN_BUS = new CANBus("Canivore");
}
