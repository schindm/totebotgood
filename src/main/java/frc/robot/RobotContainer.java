// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.swing.plaf.basic.BasicComboBoxUI.PropertyChangeHandler;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.BallAim;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.tankdrive;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.driveTrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */

public class RobotContainer {
private final XboxController m_XboxController = new XboxController(2);
  
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
private final driveTrain m_driveTrain = new driveTrain();
private final Joystick m_leftJoystick = new Joystick(1);
private final Joystick m_rightJoystick = new Joystick(0);


private final tankdrive m_tankdrive = new  tankdrive(m_driveTrain, m_leftJoystick, m_rightJoystick);
  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  private final BallAim m_ballAim = new BallAim(m_driveTrain);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    
  m_driveTrain.setDefaultCommand(m_tankdrive);
    // Configure the button bindings
    configureButtonBindings();

    

   NetworkTable ozram = NetworkTableInstance.getDefault().getTable("SmartDashboard");


// Aiming, PID variables on SmartDashboard
   NetworkTableEntry pgainEntry = ozram.getEntry("Pgain");
   pgainEntry.setNumber(.0001);
   NetworkTableEntry dgainEntry = ozram.getEntry("Dgain");
   dgainEntry.setNumber(.01);
   NetworkTableEntry timeTarget = ozram.getEntry("TargetTime");
   timeTarget.setNumber(3);
// space
   NetworkTableEntry minspeed = ozram.getEntry("minspeed");
   minspeed.setNumber(.1);


   NetworkTableEntry errorRange = ozram.getEntry("ErrorRange");
   errorRange.setNumber(1);
   

  


  }
 
  

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_XboxController, 1)
    .whenPressed(m_ballAim);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand; // no
  }
}
