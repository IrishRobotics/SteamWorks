package org.usfirst.frc.team2606.robot;

import edu.wpi.first.wpilibj.DigitalInput;
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
	public static Joystick xboxController = new Joystick(0); // Xbox Controller Joystick Object
	
	// Drive Motor Speed Controllers
	public static SpeedController frontLeftMotor = new Victor(0); // Front Left Drive Motor
	public static SpeedController frontRightMotor = new Victor(2); // Front Right Drive Motor
	public static SpeedController rearLeftMotor = new Victor(1); // Rear Left Drive Motor
	public static SpeedController rearRightMotor = new Victor(3); // Rear Right Drie Motor
	
	// Special System Motor Speed Controllers
	public static SpeedController feederMotor = new Victor(4); // Shooter Ball Feeding Motor Controller
	public static SpeedController climberMotor = new Spark(5); // Robot Climbing Motor Controller
	public static SpeedController shooterMotor = new Spark(6); // Shooter Wheel Motor Controller
	
	// Ball Shot Detection Switch
	public static DigitalInput ballShotSwitch = new DigitalInput(0); // Detects and Fires A Signal When A Ball Is Shot
	
	public static RobotDrive robotDrive = new RobotDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor); // Robot Drive Object
}
