// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.EndEffector;

public class EEShootFullSpeed extends Command {
  private EndEffector s_EndEffector;
  WaitCommand clock = new WaitCommand(Constants.EndEffector.waitTime);


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

    //put smart dashboard running command update
    SmartDashboard.putString("EE CMD", "Intake");

    //spin flywheel up to speed
    s_EndEffector.setFlywheel(s_EndEffector.calculateFlywheelVoltage(Constants.EndEffector.speakerShotRPM));
    //intake push note into flywheel if up to speed
    if(s_EndEffector.getTopMotorVelocity() >= 0.9 * Constants.EndEffector.speakerShotRPM){
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

  //finish if the note is intooken
  if(!s_EndEffector.hasNote()){
    if(!clock.isScheduled()){ //init clock if not started
      clock.initialize();
    } else { // if clock is started clock
      if(clock.isFinished()){// if clock is over end cmd and reset clock 
        clock.cancel();
        return true;
      }
    }
  }
    //finish if the note is gone
    return false;  
  }
}
