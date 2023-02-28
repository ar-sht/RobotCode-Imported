package frc.robot;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
//import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;

public class Robot extends TimedRobot {

  // Controller Assignments //

  Joystick driverController = new Joystick(0);    // Logitech Extreme 3D Pro //
  GenericHID functionController = new GenericHID(1);    // Other USB Items //
  double autoStart = 0;

  // Motor Controller Assignments //

  VictorSPX driveLeftA = new VictorSPX(4);   // VictorSPX LeftA //
  VictorSPX driveLeftB = new VictorSPX(3);   // VictorSPX LeftB //
  VictorSPX driveRightA = new VictorSPX(2);   // VictorSPX RightA //  
  VictorSPX driveRightB = new VictorSPX(1);   // VictorSPX RightB // 

  CANSparkMax leftLiftDrive = new CANSparkMax(5, MotorType.kBrushed);
  CANSparkMax rightLiftDrive = new CANSparkMax(6, MotorType.kBrushed);
  
  // Servo Assignments //

  Servo servo0 = new Servo(0);    // Servo on PWM 0 //
  //Servo servo1 = new Servo(1);    // Servo on PWM 1 //
  //Servo servo2 = new Servo(2);    // Servo on PWM 2 //

  // Acceleration Limiter //

  SlewRateLimiter filter = new SlewRateLimiter(0.5);  // SlewRateLimiter //

  @Override
  public void robotInit() {  // initialize robot //
  }
  

  @Override
  public void autonomousInit() {

    // Get a time for auton start to do events based on time later. //

    autoStart = Timer.getFPGATimestamp();

  }

  // Autonomous moves forward for 1.5 seconds at 0.5 speed then stops. //

  @Override
  public void autonomousPeriodic() {

    double autoTimeElapsed = Timer.getFPGATimestamp() - autoStart;
      
      Double autoCode = 1.0;
     
    if(autoCode == 1) {
      if(autoTimeElapsed<1.5) {

        driveLeftA.set(VictorSPXControlMode.PercentOutput, 0.5);    // Set output speed 0.5 on LeftA. //
        driveLeftB.set(VictorSPXControlMode.PercentOutput, 0.5);    // Set output speed 0.5 on LeftB. //
        driveRightA.set(VictorSPXControlMode.PercentOutput, -0.5);    // Set output speed 0.5 on RightA. //
        driveRightB.set(VictorSPXControlMode.PercentOutput, -0.5);    // Set output speed 0.5 on RightB. //

      }

      else {

        driveLeftA.set(VictorSPXControlMode.PercentOutput, 0);    // Set output speed 0 on LeftA. //
        driveLeftB.set(VictorSPXControlMode.PercentOutput, 0);    // Set output speed 0 on LeftB. //
        driveRightA.set(VictorSPXControlMode.PercentOutput, 0);    // Set output speed 0 on RightA. //
        driveRightB.set(VictorSPXControlMode.PercentOutput, 0);    // Set output speed 0 on RightB. //

      }

    } 

  }


  // This function is called once when teleop is enabled. //
  @Override
  public void teleopInit() {

    leftLiftDrive.follow(rightLiftDrive); // Tells leftLiftDrive to Follow rightLiftDrive's Position //

  }

  // This function is called periodically during operator control. //
  @Override
  public void teleopPeriodic() {

    // Driving Controls //

    double slowturn = driverController.getRawAxis(2);   // Axis 2 Slowturn //
    double forward = driverController.getRawAxis(1);    // Axis 1 Forward //
    double turn = -driverController.getRawAxis(0);    // Axis 0 Turn //
    
    driveLeftA.set(VictorSPXControlMode.PercentOutput, -(forward+(turn*0.5+(slowturn*-0.35))));   // Forward and Turn Speed at -0.5 & slowturn speed at -0.35 //
    driveLeftB.set(VictorSPXControlMode.PercentOutput, -(forward+(turn*0.5+(slowturn*-0.35))));   // Forward and Turn Speed at -0.5 & slowturn speed at -0.35 //
    driveRightA.set(VictorSPXControlMode.PercentOutput, forward-(turn*0.5-(slowturn*0.35)));    // Forward and Turn Speed at 0.5 & slowturn speed at 0.35 //
    driveRightB.set(VictorSPXControlMode.PercentOutput, forward-(turn*0.5-(slowturn*0.35)));    // Forward and Turn Speed at 0.5 & slowturn speed at 0.35 //

    // Lift Controls //

    //leftLiftDrive.setSoftLimit(SoftLimitDirection.kForward, 1080); // Set Rotation Limit 1080 Degrees Forward //
    //leftLiftDrive.setSoftLimit(SoftLimitDirection.kReverse, 1080); // Set Rotation Limit 1080 Degrees Reverse //
    //rightLiftDrive.setSoftLimit(SoftLimitDirection.kForward, 1080); // Set Rotation Limit 1080 Degrees Forward //
    //rightLiftDrive.setSoftLimit(SoftLimitDirection.kReverse, 1080); // Set Rotation Limit 1080 Degrees Reverse //

    if (driverController.getRawButtonPressed(5)) {
      //leftLiftDrive.set(0.2);
      rightLiftDrive.set(0.2);
    } else if (driverController.getRawButtonPressed(4)) {
      //leftLiftDrive.set(-0.2);
      rightLiftDrive.set(-0.2);
    } else {
      leftLiftDrive.disable();
      rightLiftDrive.disable();
    }

    if (driverController.getRawButtonPressed(11)) {
      servo0.setAngle(120);
    } else if (driverController.getRawButtonPressed(12)) {
      servo0.setAngle(0);
    } else {
      servo0.set(0);
    }


    // SlewRateLimiter //

    filter.calculate(turn);   // Slows Acceleration on turn by 0.5 //

  }
  
  // Disable Motors //

  @Override
  public void disabledInit() {
    driveLeftA.set(VictorSPXControlMode.PercentOutput,0);   // Disable Motor LeftA //
    driveLeftB.set(VictorSPXControlMode.PercentOutput,0);   // Disable Motors LeftB //
    driveRightA.set(VictorSPXControlMode.PercentOutput,0);    // Disable Motors RightA //
    driveRightB.set(VictorSPXControlMode.PercentOutput,0);    // Disable Motors RightB //

    leftLiftDrive.disable();
    rightLiftDrive.disable();
  }
}  