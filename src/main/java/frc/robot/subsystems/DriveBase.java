/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;
import frc.robot.RobotMap;
import frc.robot.commands.DriveWithJoystick;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.command.*;
import edu.wpi.first.wpilibj.Joystick;
import com.ctre.phoenix.motorcontrol.can.*;

/**
 * Add your docs here.
 */
public class DriveBase extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.


  public static WPI_TalonSRX rightTalon = RobotMap.rightFrontTalon;
  public static WPI_TalonSRX leftTalon = RobotMap.leftrontTalon;
  public static WPI_VictorSPX rightVictor = RobotMap.rightBackVictor;
  public static WPI_VictorSPX leftVictor = RobotMap.leftBackVictor;

  double rightMotorSpeed;
  double leftMotorSpeed;

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());

    setDefaultCommand(new DriveWithJoystick());
    }

  public void enableDriveBase(){

  }

  public void disableDriveBase(){
    rightTalon.disable();
    leftTalon.disable();

  }

  public void JoystickInputs(Joystick RightJoystick, Joystick LeftJoystick, Joystick Logitech){

    rightMotorSpeed = RightJoystick.getY();
    leftMotorSpeed = LeftJoystick.getY();

    rightTalon.set(rightMotorSpeed);
    leftTalon.set(leftMotorSpeed);

    rightVictor.follow(rightTalon);
    leftVictor.follow(leftTalon);

  }


}
