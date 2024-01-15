// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Vision extends SubsystemBase {
  private double angle = 0;
  private double distance = 0;
  private double targetInVision = 0;

  
  /** Creates a new Vision. */
  public Vision() {}

  //calculate turning power
  public double rotPower(double goalRotation, double currentRotation)
  {
      double error = currentRotation - goalRotation;
      double output = MathUtil.clamp(Constants.LimeLight.kP_rotate * error, -0.2, 0.2);
      return output;
  }

  //calculate pivot angle
  public double calculateGoalAngle(double distanceToTag)
  {
    return Constants.LimeLight.quadratic[0]*Math.pow(distanceToTag,2) + Constants.LimeLight.quadratic[1]*distanceToTag + Constants.LimeLight.quadratic[2];
  }

  //check if aprilTag is in shot range
  public boolean isTagInRange(double distanceToTag){
    return (distanceToTag > Constants.LimeLight.shotRange[0] && distanceToTag < Constants.LimeLight.shotRange[1]);
  }

  //get vision data atributes
  public boolean isTargetInVision(){
    return targetInVision > 0.9;
  }

  public double getAngle(){
    return angle;
  }

  public double getDistance(){
    return distance;
  }

  public double getTargetInVision(){
    return targetInVision;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //get pose of apriltag in camera space -> must configure in LL web GUI
    targetInVision = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    if(targetInVision > 0.9){
      angle = NetworkTableInstance.getDefault().getTable("limelight").getEntry("targetpose_cameraspace").getDoubleArray(new double[6])[5];
      distance = NetworkTableInstance.getDefault().getTable("limelight").getEntry("targetpose_robotspace").getDoubleArray(new double[6])[2];
    }

  }
}
