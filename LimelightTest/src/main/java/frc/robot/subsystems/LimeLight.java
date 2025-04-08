// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.annotation.Target;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimeLight extends SubsystemBase {
  /** Creates a new LimeLight. */
  private final PhotonCamera robotcamera = new PhotonCamera( "7688Camera") ;
  
  private final double CAMERA_HEIGHT = 0.026865; //height of middle of camera lens from the ground (meters)
  private final double CAMERA_PITCH = 0; //pitch angle of the camera from the horizontal plane
  private double targetHeight = 0; //height of target from ground in meters

  public LimeLight() {
    SmartDashboard.putBoolean("Target detected?", false);
    SmartDashboard.putNumber("target ID", -1);
    SmartDashboard.putNumber("target Yaw", -1);
    SmartDashboard.putNumber("target Pitch", -1);
    SmartDashboard.putNumber("estimated distance", -1);
    
  }

  public boolean CameraHasTargets() {
        var results = robotcamera.getLatestResult();        
        return results.hasTargets();
  }

  public int getTargetId()
  {
    var results = robotcamera.getLatestResult();
    PhotonTrackedTarget target = results.getBestTarget();
    if(target != null)
    {     
      return target.getFiducialId();
    }
    return -1;
  }

  public double getTargetYaw()
  {
    var results = robotcamera.getLatestResult();
    PhotonTrackedTarget target = results.getBestTarget();
    if(target != null)
    {     
      return target.getYaw();
    }
    return -1;
  }

  public double getTargetPitch()
  {
    var results = robotcamera.getLatestResult();
    PhotonTrackedTarget target = results.getBestTarget();
    if(target != null)
    {     
      return target.getPitch();
    }
    return -1;
  }

  //returns the estimated distance horizontal distance from the camera to the target (in meters)
  public double getEstimatedDistance(double targetHeight)
  {
    return PhotonUtils.calculateDistanceToTargetMeters(
      CAMERA_HEIGHT, 
      targetHeight, 
      Units.degreesToRadians(CAMERA_PITCH), 
      Units.degreesToRadians(getTargetPitch()));
  }


  @Override
  public void periodic() {
    var res = robotcamera.getLatestResult();
    PhotonTrackedTarget target = res.getBestTarget();
    Transform3d targetData = target.getBestCameraToTarget();
    if(res.hasTargets())
    {
      SmartDashboard.putBoolean("Target detected?", false);
      SmartDashboard.putNumber("target ID", getTargetId());
      SmartDashboard.putNumber("estX", Units.metersToInches(targetData.getX()));
      SmartDashboard.putBoolean("Move back", Units.metersToInches(targetData.getX()) + 1 < 40);
      SmartDashboard.putBoolean("Move forward", Units.metersToInches(targetData.getX()) - 1 > 40);

      SmartDashboard.putNumber("target Yaw", getTargetYaw());
      SmartDashboard.putNumber("target Pitch", getTargetPitch());
      SmartDashboard.putNumber("estimated distance", getEstimatedDistance(targetHeight));
    }
    
    // This method will be called once per scheduler run
  }
}
