// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.EndEffector;

public class EEShootFullSpeed extends Command {
  private EndEffector s_EndEffector;

  /** Creates a new ShootFullSpeed. */
  public EEShootFullSpeed(EndEffector endEffector) {
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
    //spin flywheel up to speed
    s_EndEffector.setFlywheel(s_EndEffector.calculateFlywheelVoltage(Constants.EndEffector.speakerShotRPM));
    //intake push note into flywheel if up to speed
    if(s_EndEffector.getMotorVelocity() >= 0.9 * Constants.EndEffector.speakerShotRPM){
      s_EndEffector.setIntake(Constants.EndEffector.passthroughPower);
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
    //finish if the note is gone
    return !s_EndEffector.hasNote();  
  }
}
