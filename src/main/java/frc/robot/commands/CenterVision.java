// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class CenterVision extends Command {
  private Swerve s_Swerve;   
  private double goalAngle; 


  /** Creates a new CenterVision. */
  public CenterVision(Swerve s_Swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);
 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    goalAngle = s_Swerve.gyroYawDouble() - s_Swerve.visionAngleError();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //put smart dashboard running command update
    SmartDashboard.putString("Swerve CMD", "Center");

        /* Get Values, Deadband*/
        double translationVal =0;
        double strafeVal = 0;
        double rotationVal = MathUtil.applyDeadband(s_Swerve.getRotPwr(goalAngle), 0.0);
        

        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
            false, 
            true
        );

        //System.out.println(goalAngle - s_Swerve.gyroYawDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    /* Drive */
        s_Swerve.drive(
            new Translation2d(0, 0).times(Constants.Swerve.maxSpeed), 
            0 * Constants.Swerve.maxAngularVelocity, 
            false, 
            true
        );
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
