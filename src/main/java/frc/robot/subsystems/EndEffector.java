// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BeamBreak;
import frc.robot.Constants;

public class EndEffector extends SubsystemBase {

  
  public static final double kS_Flywheel = 0; 
  public static final double kV_Flywheel = 0;
  public static final double kA_Flywheel = 0;
  CANSparkMax flywheelNeo1 = new CANSparkMax(Constants.EndEffector.flywheelMotor1, MotorType.kBrushless);
  CANSparkMax flywheelNeo2 = new CANSparkMax(Constants.EndEffector.flywheelMotor2, MotorType.kBrushless);
  CANSparkMax intakeNeo = new CANSparkMax(Constants.EndEffector.intakeMotor, MotorType.kBrushless);
  SimpleMotorFeedforward flywheelFeedForward = new SimpleMotorFeedforward(kS_Flywheel, kV_Flywheel, kA_Flywheel);
  BangBangController controller = new BangBangController();
  private RelativeEncoder flywheelEncoder;
  private double flywheelFeedForwardPercentage = 0.92;

  private BeamBreak beamBreak;


  /** Creates a new EndEffector. */
  public EndEffector(BeamBreak beambreak) {
    this.beamBreak = beambreak;
    flywheelNeo1.setIdleMode(IdleMode.kCoast);
    flywheelNeo2.setIdleMode(IdleMode.kCoast);
    intakeNeo.setIdleMode(IdleMode.kBrake);
    flywheelEncoder = flywheelNeo1.getEncoder();

  }



  //calculate bang-bang output for flywheel
  public double calculateFlywheelVoltage(double setpoint){
    double voltage = (controller.calculate(getMotorVelocity(), setpoint) * 12.0 + flywheelFeedForwardPercentage * flywheelFeedForward.calculate(setpoint));
    return voltage;
  }

  public double getMotorVelocity(){
    return flywheelEncoder.getVelocity(); // return flywheel motor velocity
  }

  //set power to motors
  public void setFlywheel(double v)
  {
    flywheelNeo1.setVoltage(v);
    flywheelNeo2.setVoltage(v);
  }

  public void setIntake(double p){
    intakeNeo.set(p);
  }

  public boolean hasNote(){
    return beamBreak.hasNote();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
