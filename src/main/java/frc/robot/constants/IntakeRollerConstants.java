package frc.robot.constants;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

public class IntakeRollerConstants {
    public static final int MOTOR_ID = 18;
    public static final int FOLLOWER_MOTOR_ID = 24;
    public static final double STATOR_CURRENT_LIMIT = 75;
    public static final double SUPPLY_CURRENT_LIMIT = 75;
    public static final boolean INVERTED = false;
    public static final boolean BRAKE = false;
    public static final boolean FOLLOWER_INVERTED = true;

    public static final Slot0Configs SLOT0_CONFIGS = new Slot0Configs()
            .withKP(0.25)
            .withKI(0)
            .withKD(0)
            .withKS(0.37)
            .withKG(0)
            .withKV(.1010101010101010101010101010101)
            .withKA(0)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);

    public static final double ACCELERATION = 400;
    public static final double JERK = 4000;

    public static final double LOWER_TOLERANCE = 2;
    public static final double UPPER_TOLERANCE = 2;

    public static final double[][] GEAR_RATIO = {{1, 1}};

    public static final double INTAKE_SPEED = 116;

    public static final CANBus CAN_BUS = new CANBus("rio");
}
