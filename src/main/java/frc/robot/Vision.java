// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;

public class Vision {
  private double angle = 0;
  private double distance = 0;
  private double targetInVision = 0;
  private double[] defaultReturn = new double[6];

  
  /** Creates a new Vision. */
  public Vision() {}

  //valid target
  public boolean validTarget(){
    if(isTargetInVision()){
      if(isTagInRange(getDistance())){
        return true;
      }
    }
    return false;
  }

  //calculate pivot angle
  public double calculateGoalAngle()
  {
    return Constants.LimeLight.quadratic[0]*Math.pow(getDistance(),2) + Constants.LimeLight.quadratic[1]*getDistance() + Constants.LimeLight.quadratic[2];
  }

  //check if aprilTag is in shot range
  public boolean isTagInRange(double distanceToTag){
    return (distanceToTag >= Constants.LimeLight.shotRange[0] && distanceToTag <= Constants.LimeLight.shotRange[1]);
  }

  //get vision data atributes
  public boolean isTargetInVision(){
    fetchData();
    return targetInVision > 0.9;
  }

  public double getAngle(){
    fetchData();
    return angle;
  }

  public double getDistance(){
    fetchData();
    return distance;
  }

  public double getTargetInVision(){
    fetchData();
    return targetInVision;
  }

  public void fetchData() {

    //get pose of apriltag in camera space -> must configure in LL web GUI
    targetInVision = NetworkTableInstance.getDefault().getTable(Constants.LimeLight.llTableName).getEntry(Constants.LimeLight.targetInVisionKey).getDouble(0);

    angle = NetworkTableInstance.getDefault().getTable(Constants.LimeLight.llTableName).getEntry(Constants.LimeLight.targetInAngleKey).getDouble(0);
    double[] lltable =  NetworkTableInstance.getDefault().getTable(Constants.LimeLight.llTableName).getEntry(Constants.LimeLight.targetPoseRobotSpaceKey).getDoubleArray(defaultReturn);
    distance = Math.sqrt(Math.pow(lltable[0],2) + Math.pow(lltable[2],2));
    


  }
}
