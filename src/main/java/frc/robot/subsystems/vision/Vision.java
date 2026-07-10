package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {

    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs  = VecBuilder.fill(0.5, 0.5, 1);

    private record CameraEntry(PhotonCamera camera, PhotonPoseEstimator estimator, String name) {}

    private final List<CameraEntry> cameras = new ArrayList<>();

    private final AprilTagFieldLayout fieldLayout;
    private final EstimateConsumer estimateConsumer;

    public Vision(EstimateConsumer estimateConsumer) {
        this.estimateConsumer = estimateConsumer;

        fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

        cameras.add(new CameraEntry(
            new PhotonCamera("Left"),
            new PhotonPoseEstimator(fieldLayout, new Transform3d(
                new Translation3d(VisionConstants.CameraLeft.X, VisionConstants.CameraLeft.Y, VisionConstants.CameraLeft.Z),
                new Rotation3d(VisionConstants.CameraLeft.ROLL, VisionConstants.CameraLeft.PITCH, VisionConstants.CameraLeft.YAW)
            )),
            "Left"
        ));

        cameras.add(new CameraEntry(
            new PhotonCamera("Right"),
            new PhotonPoseEstimator(fieldLayout, new Transform3d(
                new Translation3d(VisionConstants.CameraRight.X, VisionConstants.CameraRight.Y, VisionConstants.CameraRight.Z),
                new Rotation3d(VisionConstants.CameraRight.ROLL, VisionConstants.CameraRight.PITCH, VisionConstants.CameraRight.YAW)
            )),
            "Right"
        ));
    }

    @Override
    public void periodic() {
        for (var cam : cameras) {
            processCamera(cam);
        }
    }

    private void processCamera(CameraEntry cam) {
        for (var result : cam.camera().getAllUnreadResults()) {
            // try multi-tag first, fall back to single-tag if unavailable or failed
            Optional<EstimatedRobotPose> est = result.getMultiTagResult().isPresent()
                ? cam.estimator().estimateCoprocMultiTagPose(result)
                : Optional.empty();

            if (est.isEmpty()) {
                est = cam.estimator().estimateLowestAmbiguityPose(result);
            }

            est.ifPresent(pose -> {
                Matrix<N3, N1> stdDevs = getStdDevs(pose, pose.targetsUsed);
                estimateConsumer.accept(pose.estimatedPose.toPose2d(), pose.timestampSeconds, stdDevs);

                SmartDashboard.putNumber("Vision/" + cam.name() + "/X",       pose.estimatedPose.getX());
                SmartDashboard.putNumber("Vision/" + cam.name() + "/Y",       pose.estimatedPose.getY());
                SmartDashboard.putNumber("Vision/" + cam.name() + "/Heading", pose.estimatedPose.getRotation().toRotation2d().getDegrees());
            });
        }
    }

    private Matrix<N3, N1> getStdDevs(EstimatedRobotPose estimate, List<PhotonTrackedTarget> targets) {
        int numTags = 0;
        double avgDist = 0;

        for (var target : targets) {
            var tagPose = fieldLayout.getTagPose(target.getFiducialId());
            if (tagPose.isEmpty()) continue;
            numTags++;
            avgDist += tagPose.get().toPose2d().getTranslation()
                .getDistance(estimate.estimatedPose.toPose2d().getTranslation());
        }

        if (numTags == 0) return kSingleTagStdDevs;

        avgDist /= numTags;

        if (numTags > 1) {
            return kMultiTagStdDevs.times(1 + (avgDist * avgDist / 30));
        }

        if (avgDist > 4) {
            return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        }

        return kSingleTagStdDevs.times(1 + (avgDist * avgDist / 30));
    }

    @FunctionalInterface
    public interface EstimateConsumer {
        void accept(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs);
    }
}
