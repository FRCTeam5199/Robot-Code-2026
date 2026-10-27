package frc.robot.constants;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

public class IntakeRollerConstants {
    public static final int MOTOR_ID = 16;
    public static final int FOLLOWER_MOTOR_ID = 17;
    public static final double STATOR_CURRENT_LIMIT = 45;
    public static final double SUPPLY_CURRENT_LIMIT = 45;
    public static final boolean INVERTED = true;
    public static final boolean BRAKE = false;
    public static final boolean FOLLOWER_INVERTED = true;

    public static final Slot0Configs SLOT0_CONFIGS = new Slot0Configs()
            .withKP(.0012331)
            .withKI(0)
            .withKD(0)
            .withKS(0.055467)
            .withKG(0)
            .withKV(.11466)
            .withKA(0.1)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);

    public static final double ACCELERATION = 400;
    public static final double JERK = 4000;

    public static final double LOWER_TOLERANCE = 2;
    public static final double UPPER_TOLERANCE = 2;

    public static final double[][] GEAR_RATIO = {{1, 1}};

    public static final double INTAKE_SPEED = 100;
    public static final double OUTTAKE_SPEED = -100;

    public static final CANBus CAN_BUS = new CANBus("Canivore");
}
