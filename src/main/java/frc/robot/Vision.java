package frc.robot;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import com.ctre.phoenix6.jni.UtilsJNI;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Vision {

    public static final AprilTagFieldLayout kTagLayout =
                AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    
    public static final Transform3d kRobotToCam =
                new Transform3d(new Translation3d(VisionConstants.translationX, VisionConstants.translationY, VisionConstants.translationZ), new Rotation3d(0, VisionConstants.rotationPitch, 0));
    PhotonCamera front;
    PhotonCamera side;
    PhotonPoseEstimator photonEstimator;

    public Vision() {
        this.front = new PhotonCamera(VisionConstants.front);
        this.side = new PhotonCamera(VisionConstants.side);
        this.photonEstimator = new PhotonPoseEstimator(kTagLayout, kRobotToCam);
    }

    public void estimatePose(String cameraStr, CommandSwerveDrivetrain drivetrain){
        PhotonCamera camera;
        if(cameraStr.equals(VisionConstants.side)){
            camera = this.side;
        } else {
            camera = this.front;
        }
        
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        for(var result : camera.getAllUnreadResults()){
            visionEst = this.photonEstimator.estimateCoprocMultiTagPose(result);
            if (visionEst.isEmpty()) {
                visionEst = photonEstimator.estimateLowestAmbiguityPose(result);
            }
            
            if(visionEst.isPresent()) {
                drivetrain.addVisionMeasurement(visionEst.get().estimatedPose.toPose2d(), UtilsJNI.getCurrentTimeSeconds());
            }
            
            
        }
        
    }


}
