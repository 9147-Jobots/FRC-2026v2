// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import frc.robot.subsystems.drive.Drive;

import java.util.Arrays;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {

  /** Creates a new Vision. */
  private final PhotonCamera cameraLeft;
  private final PhotonCamera cameraRight;

  private final List<PhotonCamera> cameras;
  // private final PhotonCamera cameraRight;


  public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(0.5, 0.5, Math.toRadians(30)); // less confident
  public static final Matrix<N3, N1> kMultiTagStdDevs  = VecBuilder.fill(0.05, 0.05, Math.toRadians(5)); // more confident

  private Matrix<N3, N1> curStdDevs = kSingleTagStdDevs;
  private final Transform3d cameraLeftToRobot; //creating a new transform3d
  private final Transform3d cameraRightToRobot; //creating a new transform3d
  public static PhotonPoseEstimator leftPoseEstimator; //creating a new photon pose estimator
  public static PhotonPoseEstimator rightPoseEstimator; //creating a new photon pose estimator

  private final List<PhotonPoseEstimator> poseEstimators;

  private static Optional<EstimatedRobotPose> visionEst = Optional.empty();

  private final EstimateConsumer estimateConsumer;

  private AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField); //creating a new apriltag field layout
  
  public Vision(EstimateConsumer kEstimateConsumer, Drive drive) {
    estimateConsumer = kEstimateConsumer;

    //creating a new photon camera
    cameraLeft = new PhotonCamera("Left");
    cameraRight= new PhotonCamera("Right");

    cameras = Arrays.asList(
      cameraLeft,
      cameraRight
    );

    //creating a new transform3d
    cameraLeftToRobot = new Transform3d(
      new Translation3d(
          VisionConstants.cameraLeft.x, 
          VisionConstants.cameraLeft.y, 
          VisionConstants.cameraLeft.z), 
      new Rotation3d(
          VisionConstants.cameraLeft.pitch, 
          VisionConstants.cameraLeft.roll, 
          VisionConstants.cameraLeft.yaw
      ));
    
    cameraRightToRobot = new Transform3d(
      new Translation3d(
        VisionConstants.cameraRight.x,
        VisionConstants.cameraRight.y,
        VisionConstants.cameraRight.z),
      new Rotation3d(
        VisionConstants.cameraRight.pitch,
        VisionConstants.cameraRight.roll,
        VisionConstants.cameraRight.yaw
      )

      );
      
    //creating a new photon pose estimator
    leftPoseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.LOWEST_AMBIGUITY, cameraLeftToRobot);
    rightPoseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.LOWEST_AMBIGUITY, cameraRightToRobot);
    
    poseEstimators = Arrays.asList(
      leftPoseEstimator,
      rightPoseEstimator
      );
  }

  public static Optional<EstimatedRobotPose> GetFieldPose() {
    
    return visionEst;
  }

  // private void updateVisionEst(PhotonCamera camera, PhotonPoseEstimator poseEstimator, WelfordAccumulator accumulator) {
  //   for (var change : camera.getAllUnreadResults()) {
  //     visionEst = poseEstimator.update(change);
  //     updateEstimationStdDevs(visionEst, change.getTargets());

  //     visionEst.ifPresent(est -> {
  //       SmartDashboard.putNumber("Vision est X", est.estimatedPose.getTranslation().getX());
  //       SmartDashboard.putNumber("Vision est Y", est.estimatedPose.getTranslation().getY());
  //     });
  //   }
  // }

  // private void updateAllVisionEst() {
  //   for (int i = 0; i < cameras.size(); i++) {
  //     updateVisionEst(cameras.get(i), poseEstimators.get(i), accumulators.get(i));
  //   }
  //   return;
  // }
  
  @Override
  public void periodic() {
    // Instantiate Photon Camera with cam name
    // Pose2d mht_estimate = mht.getBestPose();
    // SmartDashboard.putBoolean("MHT exists", mht_estimate != null);
    // if (mht_estimate != null) {
    //   SmartDashboard.putNumber("MHT est X", mht_estimate.getTranslation().getX());
    //   SmartDashboard.putNumber("MHT est Y", mht_estimate.getTranslation().getY());
    //   estimateConsumer.accept(mht_estimate, Timer.getFPGATimestamp(), getEstimationStdDevs());
    // }
    // 
    // updateAllVisionEst();
    // for (int i = 0; i < cameras.size(); i++) {
    //   SmartDashboard.putNumber("Camera " + i + " accumulator mean", accumulators.get(i).getMean());
    //   SmartDashboard.putNumber("Camera " + i + " accumulator count", accumulators.get(i).getCount());
    //   SmartDashboard.putNumber("Camera " + i + " accumulator variance", accumulators.get(i).getVariance());
    // }
    

    if (!visionEst.isEmpty()) {
      SmartDashboard.putNumber("Pose : X", visionEst.get().estimatedPose.getTranslation().getX());
      SmartDashboard.putNumber("Pose : Y", visionEst.get().estimatedPose.getTranslation().getY());
      SmartDashboard.putNumber("Pose : Z", visionEst.get().estimatedPose.getTranslation().getZ());
      SmartDashboard.putNumber("Pose : Rotation", visionEst.get().estimatedPose.getRotation().getAngle());
      SmartDashboard.putBoolean("Vision status", true);
    } else {
      SmartDashboard.putBoolean("Vision status", false);
    }
  }

  private void updateEstimationStdDevs(
    Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
    if (estimatedPose.isEmpty()) {
    // No pose input. Default to single-tag std devs
      curStdDevs = kSingleTagStdDevs;

        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = kSingleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an average-distance metric
            for (var tgt : targets) {
                var tagPose = leftPoseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty()) continue;
                numTags++;
                avgDist +=
                        tagPose
                                .get()
                                .toPose2d()
                                .getTranslation()
                                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                curStdDevs = kSingleTagStdDevs;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1) estStdDevs = kMultiTagStdDevs;
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                curStdDevs = estStdDevs;
            }
        }
      }

    /**
     * Returns the latest standard deviations of the estimated pose from {@link
     * #getEstimatedGlobalPose()}, for use with {@link
     * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}. This should
     * only be used when there are targets visible.
     */
    public Matrix<N3, N1> getEstimationStdDevs() {
        return curStdDevs;
    }

    @FunctionalInterface
    public static interface EstimateConsumer {
        public void accept(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs);
    }
}
