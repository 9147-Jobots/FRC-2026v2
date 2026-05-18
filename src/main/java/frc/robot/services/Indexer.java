package frc.robot.services;

import frc.robot.subsystems.indexer.IndexerSubsystem;

public class Indexer {
    public void runIndexer(IndexerSubsystem indexer) {
        indexer.runVelocity(0); // TO BE TUNED
    }

    public void stopIndexer(IndexerSubsystem indexer) {
        indexer.runVelocity(0);
    }
}
