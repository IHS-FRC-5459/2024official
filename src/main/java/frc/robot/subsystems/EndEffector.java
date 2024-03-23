// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BeamBreak;
import frc.robot.Constants;

public class EndEffector extends SubsystemBase {

  TalonFX flywheelTop = new TalonFX(Constants.EndEffector.flywheelMotor1);
  TalonFX flywheelBottom = new TalonFX(Constants.EndEffector.flywheelMotor2);
  TalonFX intakeFalcon = new TalonFX(Constants.EndEffector.intakeMotor);

  private BeamBreak beamBreak;

  final VelocityVoltage m_velocityTop = new VelocityVoltage(0);
  final VelocityVoltage m_velocityBottom = new VelocityVoltage(0);
  



  /** Creates a new EndEffector. */
  public EndEffector(BeamBreak beambreak) {
    this.beamBreak = beambreak;
    flywheelTop.setNeutralMode(NeutralModeValue.Coast);
    flywheelBottom.setNeutralMode(NeutralModeValue.Coast);
    flywheelTop.setInverted(true);
    flywheelBottom.setInverted(true);
    intakeFalcon.setNeutralMode(NeutralModeValue.Brake);
    intakeFalcon.setInverted(true);

    var slot0Configs = new Slot0Configs();
    var slot1Configs = new Slot1Configs();

    slot0Configs.kV = 0.11963;
    slot1Configs.kS = 0.096037;
    slot1Configs.kA = 0.011707;

    slot0Configs.kP = 0.090179;

    flywheelTop.getConfigurator().apply(slot0Configs, 0.050);


    slot1Configs.kV = 0.11885;
    slot1Configs.kS = 0.13919;
    slot1Configs.kA = 0.011095;

    slot1Configs.kP = 0.09285;

    flywheelBottom.getConfigurator().apply(slot1Configs, 0.050);


  }



  public void setVelocity(double setpoint){
    m_velocityTop.Slot = 0;
    m_velocityBottom.Slot = 1;
    flywheelTop.setControl(m_velocityTop.withVelocity(setpoint));
    flywheelBottom.setControl(m_velocityBottom.withVelocity(setpoint));

  }

  public double getTopMotorVelocity(){
    
    return flywheelTop.getVelocity().getValueAsDouble(); // return flywheel motor velocity
  }

  public double getBottomMotorVelocity(){
    return flywheelBottom.getVelocity().getValueAsDouble(); // return flywheel motor velocity
  }

   public double getIntakeVelocity(){
    return intakeFalcon.getVelocity().getValueAsDouble(); // return flywheel motor velocity
  }


  //set power to motors from array
  public void setFlywheel(double[] v)
  {
    flywheelTop.setVoltage(v[0]);
    flywheelBottom.setVoltage(v[1]);
  }

    //set power to motors global voltage
  public void setFlywheel(double v)
  {
    flywheelTop.setVoltage(v);
    flywheelBottom.setVoltage(v);
  }

  public void setIntake(double p){
    intakeFalcon.set(p);
  }

  public boolean hasNote(){
    return beamBreak.hasNote();
  }

  public boolean hasNoteShooter(){
    return beamBreak.getResults().topState;
  }


  public Command EETimedShooterBuilder(Command EECommand){
    return Commands.race(
      EECommand,
      Commands.waitUntil(() -> !hasNote()).andThen(Commands.waitSeconds(Constants.EndEffector.waitTime))
    ); 
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("bottom", getBottomMotorVelocity());
    SmartDashboard.putNumber("top", getTopMotorVelocity());
    SmartDashboard.putNumber("intake", getIntakeVelocity());

  }
}
