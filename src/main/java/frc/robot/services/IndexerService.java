package frc.robot.services;

import frc.robot.subsystems.indexer.IndexerSubsystem;

public class IndexerService {
    public static void runIndexer(IndexerSubsystem indexer) {
        indexer.runVelocity(5); // TO BE TUNED
    }

    public static void stopIndexer(IndexerSubsystem indexer) {
        indexer.runVelocity(0);
    }
}
