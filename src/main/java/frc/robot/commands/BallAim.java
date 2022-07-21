// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.lang.reflect.Array;
import java.math.BigInteger;
import java.net.PortUnreachableException;
import java.util.FormatFlagsConversionMismatchException;

import com.fasterxml.jackson.databind.introspect.DefaultAccessorNamingStrategy;

import org.ejml.equation.Variable;
import org.ejml.equation.VariableDouble;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.driveTrain;

public class BallAim extends CommandBase {
  /** Creates a new BallAim. */
Variable One;
driveTrain m_driveTrain;
double[] areas;
double[]  defaultList= {};
int biggest;
NetworkTable table = NetworkTableInstance.getDefault().getTable("myContoursReport");
NetworkTableEntry centerX = table.getEntry("centerX");
NetworkTableEntry area = table.getEntry("area");
double[] Error;
 double pgain ;
 double minspeed;
double BigError;
double reesponse;
NetworkTable table2 = NetworkTableInstance.getDefault().getTable("SmartDashboard");
  public BallAim(
    driveTrain driveTrain
  ) {
    m_driveTrain = driveTrain;
addRequirements(m_driveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pgain = table2.getEntry("Pgain").getDouble(.0001);
  
    minspeed = table2.getEntry("minspeed").getDouble(.1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
areas =  area.getDoubleArray(defaultList)  ;

Error =  centerX.getDoubleArray(defaultList)  ;




biggest = 0;
//gets the biggest value of the list and the values identification number 
for(int listrun = 0; listrun <= areas.length-1; listrun++){
  if(areas[biggest] <= areas[listrun]){
    biggest = listrun;
  }
}
 




//cool <insert sun emoji>
// test put id to smartsdashboard
SmartDashboard.putNumber( "biggest", biggest);
 BigError  = Error[biggest] - 160   ; 
SmartDashboard.putNumber("error but big", BigError);

reesponse = pgain*BigError + minspeed*Math.signum(BigError);


m_driveTrain.drive(reesponse, reesponse );


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {m_driveTrain.drive(0,0);
  
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    if(BigError >= -1 && BigError <= 1){
      return true;

} return false;

  }
}
