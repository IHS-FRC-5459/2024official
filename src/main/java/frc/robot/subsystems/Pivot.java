// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Vision;

public class Pivot extends SubsystemBase {

  // create neo for pivot
  CANSparkMax armRotationNeo = new CANSparkMax(Constants.Arm.armRotateMotorID, MotorType.kBrushless);
  //create absolute encoder (rev through-bore)
  DutyCycleEncoder encoder = new DutyCycleEncoder(9); //TODO: SET CHANNEL

  private Vision m_Vision;

  
  /** Creates a new Pivot. */
  public Pivot(Vision vision) {
    m_Vision = vision;
    armRotationNeo.setIdleMode(IdleMode.kBrake);
    armRotationNeo.setInverted(true);
    encoder.setPositionOffset(0.21);
   
    
  }
  

  public double getAngle(){
    return (encoder.getAbsolutePosition() - encoder.getPositionOffset()) * 360;
  }


  public double calculateRotationVoltage(double goalAngle){ //TOOD: CHANGE VALUES
    return ((-1*MathUtil.clamp((Constants.Arm.kP_rotate * (getAngle() - goalAngle) * 12),-0.5,0.5)) + 0.9 * (Constants.Arm.kF_rotate * Math.cos(Math.toRadians(getAngle()))));
  }
//MathUtil.clamp((Constants.Arm.kP_rotate * (getAngle() - goalAngle) * 12),-1,1) + 
  public void setRotationVoltage(double volts){
    armRotationNeo.setVoltage(volts);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //System.out.println(getAngle() + " " +     encoder.getPositionOffset());
    //System.out.println(calculateRotationVoltage(0));
    //setRotationVoltage(calculateRotationVoltage(0));
    //System.out.println(getAngle()+ " " + (-1*MathUtil.clamp((Constants.Arm.kP_rotate * (getAngle() - 0) * 12),-0.5,0.5)));
  }

  //from vision:
  public double calculateGoalAngle(){
    return m_Vision.calculateGoalAngle();
  }


}
