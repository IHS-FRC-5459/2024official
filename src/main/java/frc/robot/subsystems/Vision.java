// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Calendar;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.LimeLight;

public class Vision extends SubsystemBase{
  private double angle = 0;
  private double distance = 0;
  private double targetInVision = 0;
  private double[] defaultReturn = new double[6];
  private double default_ = 0.0;
  private double lastValue = default_;
  private Calendar lastTime = Calendar.getInstance();
  private double duration =  0.5 * 1000;/*secondsToMiliseconds(1)*/
  private Calendar testingCalendar = Calendar.getInstance();
  private double myNewVal = default_;
  /** Creates a new Vision. */
  public Vision() {
    testingCalendar.add(Calendar.SECOND, 20);
  }

  //valid target
  public boolean validTarget(){
    if(isTargetInVision()){
      if(isTagInRange(getRangeFromCache())){
        return true;
      }
    }
    return false;
  }

  //calculate pivot angle
  public double calculateGoalAngle()
  {
    double dist = getRangeFromCache();

        return Constants.LimeLight.cubicFit[0]*Math.pow(dist,3) + Constants.LimeLight.cubicFit[1]*Math.pow(dist,2) + Constants.LimeLight.cubicFit[2]*Math.pow(dist,1)  + Constants.LimeLight.cubicFit[3];
  }

    //calculate pivot angle
  public double calculateGoalAngle(double dist)
  {
        return Constants.LimeLight.cubicFit[0]*Math.pow(dist,3) + Constants.LimeLight.cubicFit[1]*Math.pow(dist,2) + Constants.LimeLight.cubicFit[2]*Math.pow(dist,1)  + Constants.LimeLight.cubicFit[3];
}

  //check if aprilTag is in shot range
  public boolean isTagInRange(double distanceToTag){
    return (distanceToTag >= Constants.LimeLight.shotRange[0] && distanceToTag <= Constants.LimeLight.shotRange[1]);
  }

  //get vision data atributes
  public boolean isTargetInVision(){
    fetchData();
    return targetInVision > 0.9 /*getRangeFromCache() > default_*/;
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
  private void updateCache(){
    fetchData();
    this.myNewVal = getDistance();
        if(this.myNewVal > this.default_){
            this.lastTime = Calendar.getInstance();
            this.lastValue = this.myNewVal;
      }
  }
  public double getRangeFromCache(){
    if(Calendar.getInstance().compareTo(this.lastTime) <= this.duration){
        return this.lastValue;
    }
  return this.default_;
}
@Override
public void periodic(){
   updateCache();
  }
}
