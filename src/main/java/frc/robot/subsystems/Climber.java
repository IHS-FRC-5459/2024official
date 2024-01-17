// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.invoke.ConstantBootstraps;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  TalonFX climberMotor = new TalonFX(Constants.Climber.climberMotorID);
  /** Creates a new Climber. */
  public Climber() {
    climberMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public void setClimberPower(double p){
    climberMotor.set(p);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
