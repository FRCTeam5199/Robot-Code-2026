package frc.robot.constants;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

public class HopperConstants {
    public static final int MOTOR_ID = 14;
    public static final double STATOR_CURRENT_LIMIT = 50;
    public static final double SUPPLY_CURRENT_LIMIT = 50;
    public static final boolean INVERTED = false;
    public static final boolean BRAKE = false;

    public static final Slot0Configs SLOT0_CONFIGS = new Slot0Configs()
            .withKP(0.5)
            .withKI(0)
            .withKD(0)
            .withKS(.5)
            .withKG(0)
            .withKV(.11494252873563218390804597701149)
            .withKA(0)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);

    public static final double ACCELERATION = 400;
    public static final double JERK = 4000;

    public static final double LOWER_TOLERANCE = 50;
    public static final double UPPER_TOLERANCE = 50;

    public static final double[][] GEAR_RATIO = {{1, 2.5}};

    public static final double INDEXING_SPEED = 75;
    public static final double IDLING_SPEED = -5;

    public static final CANBus canbus = new CANBus("rio");
}