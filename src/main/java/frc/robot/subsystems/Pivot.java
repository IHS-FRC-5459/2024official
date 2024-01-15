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

public class Pivot extends SubsystemBase {

  // create neo for pivot
  CANSparkMax armRotationNeo = new CANSparkMax(Constants.Arm.armRotateMotorID, MotorType.kBrushless);
  //create absolute encoder (rev through-bore)
  DutyCycleEncoder encoder = new DutyCycleEncoder(9); //TODO: SET CHANNEL

  /** Creates a new Pivot. */
  public Pivot() {
    armRotationNeo.setIdleMode(IdleMode.kBrake);
  }

  public double getAngle(){
    return encoder.getAbsolutePosition();
  }

  public double calculateRotationVoltage(double goalAngle){ //TOOD: CHANGE VALUES
    return (MathUtil.clamp((Constants.Arm.kP_rotate * (getAngle() - goalAngle) * 12),-6,6) + 0.9 * (Constants.Arm.kF_rotate * Math.cos(Math.toRadians(getAngle()))));
  }

  public void setRotationVoltage(double volts){
    armRotationNeo.setVoltage(volts);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
