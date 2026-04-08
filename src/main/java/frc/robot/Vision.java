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

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;

import com.ctre.phoenix6.Utils;

public class Vision {

    public static final AprilTagFieldLayout kTagLayout =
                AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    
    public static final Transform3d kRobotToCam =
                new Transform3d(new Translation3d(VisionConstants.translationX, VisionConstants.translationY, VisionConstants.translationZ), new Rotation3d(0, VisionConstants.rotationPitch, 0));
    PhotonCamera front;
    //PhotonCamera side;
    PhotonPoseEstimator photonEstimator;
    Field2d visionField = new Field2d();


    public Vision() {
        this.front = new PhotonCamera(VisionConstants.front);
        //this.side = new PhotonCamera(VisionConstants.side);
        this.photonEstimator = new PhotonPoseEstimator(kTagLayout, kRobotToCam);
        SmartDashboard.putData("visionField", visionField);
    }

    public Pose2d estimatePose(/*CommandSwerveDrivetrain drivetrain*/){
        PhotonCamera camera = this.front;
        /*
        if(cameraStr.equals(VisionConstants.side)){
            camera = this.side;
        } else {
            camera = this.front;
        }
        */
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        Pose2d estimatedPose = new Pose2d();
        for(var result : camera.getAllUnreadResults()){
            visionEst = this.photonEstimator.estimateCoprocMultiTagPose(result);
            if (visionEst.isEmpty()) {
                visionEst = photonEstimator.estimateLowestAmbiguityPose(result);
            }
            
            if(visionEst.isPresent()) {
                estimatedPose = visionEst.get().estimatedPose.toPose2d();
                //System.out.println(estimatedPose);
                //drivetrain.addVisionMeasurement(visionEst.get().estimatedPose.toPose2d(), Utils.fpgaToCurrentTime(visionEst.get().timestampSeconds));
                visionField.setRobotPose(estimatedPose);
            }
            
            
        }
        return estimatedPose;
        
    }
    


}
