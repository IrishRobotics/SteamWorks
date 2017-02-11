package org.usfirst.frc.team2606.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Victor;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
	
	// Xbox One Controller
	public static Joystick xboxController = new Joystick(0);;
	
	// Drive Motor Speed Controllers
	public static SpeedController frontLeftMotor = new Victor(0);
	public static SpeedController frontRightMotor = new Victor(2);
	public static SpeedController rearLeftMotor = new Victor(1);
	public static SpeedController rearRightMotor = new Victor(3);
	
	// Special System Motor Speed Controllers
	public static SpeedController climberMotor = new Spark(4); // Robot Climbing Motor Controller
	public static SpeedController shooterMotor = new Spark(5); // Shooter Wheel Motor Controller
	
	public static RobotDrive robotDrive = new RobotDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor); // Robot Drive Object
}
