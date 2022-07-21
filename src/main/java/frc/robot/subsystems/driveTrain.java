// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class driveTrain extends SubsystemBase {

TalonFX leftmotor;
TalonFX rightmotor;
TalonFX backleft;
TalonFX backright;


  /** Creates a new driveTrain. */
  public driveTrain(


  ) {



 leftmotor = new TalonFX(Constants.leftmotor) ;
rightmotor = new TalonFX(Constants.rightmotor);

backright = new TalonFX(Constants.backright);
backleft = new TalonFX(Constants.backleft);


  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
  }

public void drive(double leftmotorOut,  double rightmotorOut ){
rightmotor.set(ControlMode.PercentOutput, rightmotorOut); 
leftmotor.set(ControlMode.PercentOutput, leftmotorOut);
backright.set(ControlMode.Follower, rightmotor.getDeviceID());
backleft.set(ControlMode.Follower,leftmotor.getDeviceID() );

}


}

