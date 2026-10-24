package frc.robot.constants;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

public class KickerConstants {
    public static final int MOTOR_ID = 12;

    public static final double STATOR_CURRENT_LIMIT = 85;
    public static final double SUPPLY_CURRENT_LIMIT = 60;

    public static final boolean INVERTED = false;

    public static final boolean BRAKE = false;
    
    public static final double INDEXER_SPEED = 20;

    public static final Slot0Configs SLOT0_CONFIGS = new Slot0Configs()
            .withKP(0.4)
            .withKI(0)
            .withKD(0)
            .withKS(.45)
            .withKG(0)
            .withKV(.11111)
            .withKA(0.15)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);

    public static final double ACCELERATION = 400;
    public static final double JERK = 4000;

    public static final double LOWER_TOLERANCE = 8d;
    public static final double UPPER_TOLERANCE = 8d;

    public static final double[][] GEAR_RATIO = {{1, 1}};

    public static final CANBus canbus = new CANBus("Canivore");
}
