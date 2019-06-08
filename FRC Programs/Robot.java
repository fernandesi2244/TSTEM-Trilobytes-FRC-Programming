/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/* @author Ian Fernandes and Novice Programmers                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

//Import necessary classes to run robot hardware commands
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.cameraserver.CameraServer;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
  //Define left side of differential drive as a SpeedControllerGroup object
  Spark leftFront = new Spark(5);
  Spark leftRear = new Spark(1);
  SpeedControllerGroup left = new SpeedControllerGroup(leftFront, leftRear);

  //Define right side of differential drive as a SpeedControllerGroup object
  Spark rightFront = new Spark(2);
  Spark rightRear = new Spark(3);
  SpeedControllerGroup right = new SpeedControllerGroup(rightFront, rightRear);

  //Construct speed controller to operate rotational-action hatch motor
  PWMVictorSPX hatchMotor = new PWMVictorSPX(4);

  //Construct speed controller to operate cargo motor
  PWMVictorSPX cargoIntake = new PWMVictorSPX(0);

  //Construct DifferentialDrive object
  private final DifferentialDrive differentialDrive
      = new DifferentialDrive(left, right);

  //Construct two joysticks, one for driving, one for "arm" operations
  private final Joystick driveStick = new Joystick(0);
  private final Joystick armStick = new Joystick(1);

  //Construct boolean for button to toggle high speed/low speed for differential drive.
  private boolean highSpeed = true;

  //Constants for rotational-action hatch motor and cargo motor
  private final double ROTATIONAL_HATCH_MOTOR_POWER = .75;
  private final double CARGO_MOTOR_POWER = .3;

  private final Timer autonomousTimer = new Timer();

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    //Start camera streams to driver station.
    CameraServer.getInstance().startAutomaticCapture();
    CameraServer.getInstance().startAutomaticCapture();

    //Optional
    hatchMotor.setSafetyEnabled(true);
    cargoIntake.setSafetyEnabled(true);
  }

  /**
   * This function is run once each time the robot enters autonomous mode.
   */
  @Override
  public void autonomousInit() {
    //Set up autonomous timer.
    autonomousTimer.reset();
    autonomousTimer.start();
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    if(driveStick.getRawButton(11))
      highSpeed = !highSpeed;
    //Operate differential drive with arcade drive function using the driveStick's Y and X values respectively.
    differentialDrive.arcadeDrive((highSpeed?(-driveStick.getY()*.75):(-driveStick.getY()/4.)), driveStick.getX()*.75);

    //If the trigger on armStick is pushed, move motor forward.
    hatchMotor.set(armStick.getTrigger()?ROTATIONAL_HATCH_MOTOR_POWER:0);

    //Operate linear hatch with deadband of 0.5 and instantaneous speed
    if(armStick.getY()<-0.5)
      cargoIntake.set(CARGO_MOTOR_POWER);
    else if(armStick.getY()>0.5)
      cargoIntake.set(-CARGO_MOTOR_POWER);
    else
      cargoIntake.set(0);
  }

  /**
   * This function is called once each time the robot enters teleoperated mode.
   */
  @Override
  public void teleopInit() {
  }

  /**
   * This function is called periodically during teleoperated mode.
   */
  @Override
  public void teleopPeriodic() {
    if(driveStick.getRawButton(11))
      highSpeed = !highSpeed;
    //Operate differential drive with arcade drive function using the driveStick's Y and X values respectively.
    differentialDrive.arcadeDrive((highSpeed?(-driveStick.getY()*.75):(-driveStick.getY()/4.)), driveStick.getX()*.75);

    //If the trigger on armStick is pushed, move motor forward.
    hatchMotor.set(armStick.getTrigger()?ROTATIONAL_HATCH_MOTOR_POWER:0);

    //Operate cargo mechanism
    if(armStick.getY()<-0.5)
      cargoIntake.set(CARGO_MOTOR_POWER);
    else if(armStick.getY()>0.5)
      cargoIntake.set(-CARGO_MOTOR_POWER);
    else
      cargoIntake.set(0);
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
