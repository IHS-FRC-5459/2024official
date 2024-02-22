// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.EndEffector;

public class EESpeakerWithVision extends Command {
  private EndEffector s_EndEffector;

  /** Creates a new ShootFullSpeed. */
  public EESpeakerWithVision(EndEffector endEffector) {
    s_EndEffector = endEffector;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_EndEffector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //put smart dashboard running command update
    SmartDashboard.putString("EE CMD", "Intake");

    //spin flywheel up to speed
    s_EndEffector.setVelocity(Constants.EndEffector.speakerShotRPS);
    //intake push note into flywheel if up to speed
    if(Math.abs(s_EndEffector.getTopMotorVelocity()) >= 400){
      //s_EndEffector.setIntake(Constants.EndEffector.passthroughPower);
      s_EndEffector.setIntake(0.8);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_EndEffector.setFlywheel(0);
    s_EndEffector.setIntake(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;  
  }
}
