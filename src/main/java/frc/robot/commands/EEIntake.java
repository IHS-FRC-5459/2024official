// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffector;

public class EEIntake extends Command {
  private EndEffector s_EndEffector;

  /** Creates a new Intake. */
  public EEIntake(EndEffector endEffector) {
    s_EndEffector = endEffector;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_EndEffector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // check if note is fully intooken
    if(s_EndEffector.hasNote()){
        // stop shooter and intake
      s_EndEffector.setFlywheel(0);
      s_EndEffector.setIntake(0);


     
    } else {
       //run shooter motors slowly backwards
      s_EndEffector.setFlywheel(-1);
      // run intake
      s_EndEffector.setIntake(0.3);

    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //finish if the note is intooken
    return s_EndEffector.hasNote();
  }
}
