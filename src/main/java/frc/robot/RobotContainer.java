package frc.robot;

import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.commands.*;
import frc.robot.commands.TestLED;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick operator = new Joystick(1);


    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton shotButton = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    
    private final JoystickButton climberLockOut = new JoystickButton(operator, XboxController.Button.kY.value);
    private final JoystickButton climberDownButton = new JoystickButton(operator, XboxController.Button.kLeftBumper.value);
    private final JoystickButton climberUpButton = new JoystickButton(operator, XboxController.Button.kRightBumper.value);
    
    private final JoystickButton ampPivot = new JoystickButton(operator, XboxController.Button.kA.value);
    private final JoystickButton ampShoot = new JoystickButton(operator, XboxController.Button.kX.value);

    //testing buttons
    private final JoystickButton testLED = new JoystickButton(driver, XboxController.Button.kX.value);

    //change back to operator #TODO
    private final JoystickButton intakeButton = new JoystickButton(operator, XboxController.Button.kB.value);

    /* Sensors */
    public final int[] channels = {7,6};
    public final Vision vision = new Vision();
    public final BeamBreak beambreak = new BeamBreak(channels);
    /* Subsystems */

    public final Swerve s_Swerve = new Swerve(vision);
    private final Pivot s_Pivot = new Pivot(vision, beambreak);
    private final EndEffector s_EndEffector = new EndEffector(beambreak);
    private final Climber s_Climber = new Climber();
    private final TestingLEDSub s_Led = new TestingLEDSub();


    private final SendableChooser<Command> autoChooser;



    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        //named commands for swerve
        NamedCommands.registerCommand("intake", Commands.parallel(new EEIntake(s_EndEffector), new PivotToNeutral(s_Pivot)).withTimeout(2));//intaking
        NamedCommands.registerCommand("pivot", new PivotToNeutral(s_Pivot).withTimeout(0.6));
       // NamedCommands.registerCommand("shoot", Commands.race(Commands.waitUntil(() -> !beambreak.hasNote()),(new ParallelCommandGroup(autoShoot()))).withTimeout(4));//shooting
        NamedCommands.registerCommand("shoot", Commands.defer( ()->(Commands.race(Commands.waitUntil(() -> !beambreak.hasNote()),(new ParallelCommandGroup(autoShoot()))).withTimeout(4)), Set.of(s_EndEffector,s_Pivot)));
        //  NamedCommands.registerCommand("farshot", Commands.race(Commands.waitUntil(() -> !beambreak.hasNote()),(new ParallelCommandGroup(autoShootTwo()))).withTimeout(4));//shooting
        //NamedCommands.registerCommand("farshotLast", Commands.race(Commands.waitUntil(() -> !beambreak.hasNote()),(new ParallelCommandGroup(autoShootLast()))).withTimeout(4));//shooting

        //NamedCommands.registerCommand("turnDegrees", new SwerveToAngle());

         s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            )
        );

          s_Pivot.setDefaultCommand(
          new PivotToNeutral(
            s_Pivot
            )
      );
 
        s_Climber.setDefaultCommand(
            new ClimberDefault(s_Climber)
        );
       
        s_EndEffector.setDefaultCommand(
            new EENeutral(s_EndEffector)
       );

        

        





        // Configure the button bindings
        configureButtonBindings();

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
       // shotButton.whileTrue(testingShot(40));
      // shotButton.whileTrue(Commands.defer(() -> testingShot(34.5),Set.of(s_EndEffector,s_Pivot)));
       shotButton.whileTrue(Commands.defer(this::shootSpeaker,Set.of(s_EndEffector,s_Pivot, s_Swerve))).debounce(0.3);
       //shotButton.whileTrue(new PivotToSpeaker(s_Pivot, 35));
        intakeButton.whileTrue(new EEIntake(s_EndEffector)).debounce(0.3);
        
        climberLockOut.and(climberDownButton).whileTrue(new ClimberTranslate(s_Climber, Constants.Climber.climberDownPower));
        climberLockOut.and(climberUpButton).whileTrue((new ClimberTranslate(s_Climber, Constants.Climber.climberUpPower)));

             //   shotButton.whileTrue(shootSpeaker());
        ampPivot.whileTrue(s_Pivot.withNoteTimeout(new PivotToAmp(s_Pivot)));
        ampShoot.whileTrue(new EEShootAmpSpeed(s_EndEffector));
        
        int[] testingColor = {40,40,40};
        testLED.whileTrue(new TestLED(1, testingColor, s_Led));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public Command testingShot(double angle){
     /*      if(vision.validTarget()){
           // double goalPivotAngle = vision.calculateGoalAngle();
            double goalPivotAngle = angle;
            return Commands.parallel(
               // s_Swerve.centerVisionBuilder(),
                s_Pivot.withNoteTimeout(new PivotToSpeaker(s_Pivot, goalPivotAngle)),
                Commands.waitUntil(() -> (s_Pivot.getAngle() > goalPivotAngle - 2)).andThen(
                s_EndEffector.EETimedShooterBuilder(new EESpeakerWithVision(s_EndEffector)),
                Commands.deadline(Commands.waitUntil(() -> (s_Pivot.getAngle() > goalPivotAngle - 2)), new EESpinUp(s_EndEffector)))
            );
        } else {
        return Commands.waitSeconds(0.01);

        }*/

           return Commands.parallel(
            s_Pivot.withNoteTimeout(new PivotToSpeaker(s_Pivot, angle)),
            Commands.waitUntil(() -> (s_Pivot.getAngle() > angle - 1)).andThen(
            s_EndEffector.EETimedShooterBuilder(new EESpeakerNoVision(s_EndEffector)),
            Commands.deadline(Commands.waitUntil(() -> (s_Pivot.getAngle() > angle - 1)), new EESpinUp(s_EndEffector)))
        );
    }

    //large command builders:
    //for during teleop
      public Command shootSpeaker(){
    if(!vision.validTarget()){
        return Commands.none();
    } else {
        if(vision.getDistance() > 1.3){
        double goalPivotAngle = vision.calculateGoalAngle(vision.getDistance());
        return Commands.parallel(
                s_Swerve.centerVisionBuilder(),
                s_Pivot.withNoteTimeout(new PivotToSpeaker(s_Pivot, goalPivotAngle)),
                Commands.waitUntil(() -> (s_Pivot.getAngle() > goalPivotAngle - 1)).andThen(
                s_EndEffector.EETimedShooterBuilder(new EESpeakerWithVision(s_EndEffector)),
                Commands.deadline(Commands.waitUntil(() -> (s_Pivot.getAngle() > goalPivotAngle - 1)), new EESpinUp(s_EndEffector)))
        ); 
        } else {
                        return Commands.parallel(
            s_Pivot.withNoteTimeout(new PivotAtSubwoofer(s_Pivot)),
            Commands.waitUntil(() -> (s_Pivot.getAngle() > Constants.Arm.subwooferAngle - 1)).andThen(
            s_EndEffector.EETimedShooterBuilder(new EESpeakerNoVision(s_EndEffector)),
            Commands.deadline(Commands.waitUntil(() -> (s_Pivot.getAngle() > Constants.Arm.subwooferAngle - 1)), new EESpinUp(s_EndEffector)))
        );

        }
    }

    }



//for during auto
    public Command autoShoot(){
        if(vision.validTarget() && vision.getDistance() > 1.5){


          /*   return (Commands.race(
             (Commands.waitUntil(() -> (!beambreak.hasNote()))),
             (Commands.parallel(
                        s_Pivot.withNoteTimeout(new PivotToSpeaker(s_Pivot, goalPivotAngle)),
                        Commands.waitUntil(() -> (s_Pivot.getAngle() > goalPivotAngle - 2)).andThen(
                        s_EndEffector.EETimedShooterBuilder(new EESpeakerWithVision(s_EndEffector)),
                        Commands.deadline(Commands.waitUntil(() -> (s_Pivot.getAngle() > goalPivotAngle - 2)), new EESpinUp(s_EndEffector))
             )))).andThen(new PivotToNeutral(s_Pivot))).withTimeout(3);
*/
            
              double goalPivotAngle = vision.calculateGoalAngle(vision.getDistance());
            // System.out.println(goalPivotAngle);
            return new ParallelCommandGroup(
                        s_Pivot.withNoteTimeout(new PivotToSpeaker(s_Pivot, goalPivotAngle)),
                        Commands.waitUntil(() -> (s_Pivot.getAngle() > goalPivotAngle - 1)).andThen(
                        s_EndEffector.EETimedShooterBuilder(new EESpeakerWithVision(s_EndEffector)),
                        Commands.deadline(Commands.waitUntil(() -> (s_Pivot.getAngle() > goalPivotAngle - 1))), new EESpinUp(s_EndEffector))
        );  


        

        } else {
            //subwoofer shot sequence. no center or vision use for angle

          /*   return (Commands.race(
             (Commands.waitUntil(() -> (!beambreak.hasNote()))),
             (
                Commands.parallel(
            s_Pivot.withNoteTimeout(new PivotAtSubwoofer(s_Pivot)),
            Commands.waitUntil(() -> (s_Pivot.getAngle() > Constants.Arm.subwooferAngle - 2)).andThen(
            s_EndEffector.EETimedShooterBuilder(new EESpeakerNoVision(s_EndEffector)),
            Commands.deadline(Commands.waitUntil(() -> (s_Pivot.getAngle() > Constants.Arm.subwooferAngle - 2))), new EESpinUp(s_EndEffector))))).andThen(new PivotToNeutral(s_Pivot))).withTimeout(3);
*/          
                   //       System.out.println("auto shoot subwoofer");

            return new ParallelCommandGroup((Commands.parallel(
            s_Pivot.withNoteTimeout(new PivotAtSubwoofer(s_Pivot)),
            Commands.waitUntil(() -> (s_Pivot.getAngle() > Constants.Arm.subwooferAngle - 1)).andThen(
            s_EndEffector.EETimedShooterBuilder(new EESpeakerNoVision(s_EndEffector)),
            Commands.deadline(Commands.waitUntil(() -> (s_Pivot.getAngle() > Constants.Arm.subwooferAngle - 1))), new EESpinUp(s_EndEffector))
            )).withTimeout(3));
        }
    }

    public Command autoShootTwo(){
        double angle = 35;
                    return new ParallelCommandGroup(
                        s_Pivot.withNoteTimeout(new PivotToSpeaker(s_Pivot, angle)),
                        Commands.waitUntil(() -> (s_Pivot.getAngle() > angle - 1)).andThen(
                        s_EndEffector.EETimedShooterBuilder(new EESpeakerWithVision(s_EndEffector)),
                        Commands.deadline(Commands.waitUntil(() -> (s_Pivot.getAngle() > angle - 1))), new EESpinUp(s_EndEffector))
        );  

        

    }

     public Command autoShootLast(){
        double angle = 36.5;
                    return new ParallelCommandGroup(
                        s_Pivot.withNoteTimeout(new PivotToSpeaker(s_Pivot, angle)),
                        Commands.waitUntil(() -> (s_Pivot.getAngle() > angle - 1)).andThen(
                        s_EndEffector.EETimedShooterBuilder(new EESpeakerWithVision(s_EndEffector)),
                        Commands.deadline(Commands.waitUntil(() -> (s_Pivot.getAngle() > angle - 1))), new EESpinUp(s_EndEffector))
        );  

        

    }

/* 

    public Command shootWithCenterSpeaker(){
        //used in auto 
        return Commands.parallel(
            s_Swerve.centerVisionBuilder().andThen(s_EndEffector.EETimedShooterBuilder(new EEShootFullSpeed(s_EndEffector))),
            s_Pivot.withNoteTimeout(new PivotToAmp(s_Pivot))
        );
    }*/
}
