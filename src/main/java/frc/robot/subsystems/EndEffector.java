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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BeamBreak;
import frc.robot.Constants;

public class EndEffector extends SubsystemBase {

  
  public static final double kS_FlywheelTop = 0; 
  public static final double kV_FlywheelTop = 0;
  public static final double kA_FlywheelTop = 0;
  
  public static final double kS_FlywheelBottom = 0; 
  public static final double kV_FlywheelBottom = 0;
  public static final double kA_FlywheelBottom = 0;

  CANSparkMax flywheelNeoTop = new CANSparkMax(Constants.EndEffector.flywheelMotor1, MotorType.kBrushless);
  CANSparkMax flywheelNeoBottom = new CANSparkMax(Constants.EndEffector.flywheelMotor2, MotorType.kBrushless);
  CANSparkMax intakeNeo = new CANSparkMax(Constants.EndEffector.intakeMotor, MotorType.kBrushless);
  SimpleMotorFeedforward flywheelTopFeedForward = new SimpleMotorFeedforward(kS_FlywheelTop, kV_FlywheelTop, kA_FlywheelTop);
  SimpleMotorFeedforward flywheelBottomFeedForward = new SimpleMotorFeedforward(kS_FlywheelBottom, kV_FlywheelBottom, kA_FlywheelBottom);

  BangBangController controller = new BangBangController();
  private RelativeEncoder flywheelTopEncoder;
  private RelativeEncoder flywheelBottomEncoder;
  private RelativeEncoder intakeEncoder;

  private double flywheelTopFeedForwardPercentage = 0.92;
  private double flywheelBottomFeedForwardPercentage = 0.92;


  private BeamBreak beamBreak;


  /** Creates a new EndEffector. */
  public EndEffector(BeamBreak beambreak) {
    this.beamBreak = beambreak;
    flywheelNeoTop.setIdleMode(IdleMode.kCoast);
    flywheelNeoBottom.setIdleMode(IdleMode.kCoast);
    flywheelNeoTop.setInverted(true);
    flywheelNeoBottom.setInverted(true);
    intakeNeo.setIdleMode(IdleMode.kBrake);
    intakeNeo.setInverted(true);
    flywheelTopEncoder = flywheelNeoTop.getEncoder();
    flywheelBottomEncoder = flywheelNeoBottom.getEncoder();
    intakeEncoder = intakeNeo.getEncoder();


  }



  //calculate bang-bang output for flywheel
  public double[] calculateFlywheelVoltage(double setpoint){
    double voltage[] = {0,0};
    //double[] voltage = {(controller.calculate(getTopMotorVelocity(), setpoint) * 12.0 + flywheelTopFeedForwardPercentage * flywheelTopFeedForward.calculate(setpoint)), controller.calculate(getBottomMotorVelocity(), setpoint) * 12.0 + flywheelBottomFeedForwardPercentage * flywheelBottomFeedForward.calculate(setpoint)};
    if(setpoint != 0){
      double fullVoltage[] = {9,9};
      return fullVoltage;
    } 


    return voltage;
  }

  public double getTopMotorVelocity(){
    return flywheelTopEncoder.getVelocity(); // return flywheel motor velocity
  }

  public double getBottomMotorVelocity(){
    return flywheelBottomEncoder.getVelocity(); // return flywheel motor velocity
  }

   public double getIntakeVelocity(){
    return intakeEncoder.getVelocity(); // return flywheel motor velocity
  }


  //set power to motors from array
  public void setFlywheel(double[] v)
  {
    flywheelNeoTop.setVoltage(v[0]);
    flywheelNeoBottom.setVoltage(v[1]);
  }

    //set power to motors global voltage
  public void setFlywheel(double v)
  {
    flywheelNeoTop.setVoltage(v);
    flywheelNeoBottom.setVoltage(v);
  }

  public void setIntake(double p){
    intakeNeo.set(p);
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
