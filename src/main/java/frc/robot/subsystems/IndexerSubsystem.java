package frc.robot.subsystems;

import frc.robot.constants.IndexerConstants;
import frc.robot.subsystems.templates.TemplateSubsystem;
import frc.robot.utility.Type;

public class IndexerSubsystem extends TemplateSubsystem {
    private static IndexerSubsystem indexerSubsystem;

    private IndexerSubsystem() {
        super(Type.ROLLER, IndexerConstants.INDEXER_MOTOR_ID,
                0, IndexerConstants.INDEXER_ACCELERATION,
                IndexerConstants.INDEXER_JERK,
                IndexerConstants.INDEXER_LOWER_TOLERANCE,
                IndexerConstants.INDEXER_UPPER_TOLERANCE,
                IndexerConstants.INDEXER_GEAR_RATIO, "INDEXER");

        configureMotor(IndexerConstants.INDEXER_INVERTED, IndexerConstants.INDEXER_BRAKE,
                IndexerConstants.INDEXER_SUPPLY_CURRENT_LIMIT,
                IndexerConstants.INDEXER_STATOR_CURRENT_LIMIT,
                IndexerConstants.INDEXER_SLOT0_CONFIGS);
    }

    public static IndexerSubsystem getInstance() {
        if (indexerSubsystem == null) {
            indexerSubsystem = new IndexerSubsystem();
        }
        return indexerSubsystem;
    }

    @Override
    public void periodic() {
        super.periodic();
//        System.out.println("Velocity: " + getMotorVelocity());
    }
}
