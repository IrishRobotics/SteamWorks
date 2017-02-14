package org.usfirst.frc.team2606.robot;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.vision.VisionThread;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.usfirst.frc.team2606.robot.commands.AutonomousDriveForward;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {

	public static int autonomousLoopCounter;
	public static Command autonomousCommand;

	// Default D-Pad Control Drive Speed 0 - 100%
	private static double DEFAULT_DPAD_SPEED;
	
	// Shooter Hold Speed Steady Button
	private static boolean isShooterHoldSteady;

	// Shooter Additional Velocity After Shooting (Ramp Back Up To Speed)
	private static int shooterAdditionalVelocity;
	private static int shooterAdditionalVelocityCounter;
	private static double shooterAdditionalVelocityMultiplier;
	
	// Default Shooter Speed 0 - 100%
	private static double DEFAULT_MIN_SHOOTER_SPEED;

	// Shooter Hold Steady Speed
	private static double shooterHoldSteadySpeed;

	// Size of camera image
	private static int IMG_WIDTH;
	private static int IMG_HEIGHT;

	// Center of Image & Thread Lock object to used when obtaining the image
	private double centerX = 0.0;
	private final Object imgLock = new Object();
	
	private static final boolean ENABLE_VISION = false;

	SendableChooser<Command> autonomousRoutinePicker = new SendableChooser<>();

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		// Create Commands For Different Autonomous Routines
		autonomousRoutinePicker.addDefault("Drive Forward & Stop", new AutonomousDriveForward());
		autonomousRoutinePicker.addObject("Test 1", new AutonomousDriveForward());
		autonomousRoutinePicker.addObject("Test 2", new AutonomousDriveForward());
		SmartDashboard.putData("Autonomous Mode Command", autonomousRoutinePicker);

		// Default D-Pad Drive Speed
		DEFAULT_DPAD_SPEED = 0.5;
		
		// Shooter Default Speed
		DEFAULT_MIN_SHOOTER_SPEED = 0.55;
		isShooterHoldSteady = false;

		// Shooter Additional Velocity After Firing
		shooterAdditionalVelocity = 30;
		shooterAdditionalVelocityCounter = 0;
		shooterAdditionalVelocityMultiplier = 0.3;
		
		// Vision Code
		if (ENABLE_VISION) {
		IMG_WIDTH = 640;
		IMG_HEIGHT = 480;

		// Raw unchanged camera
		UsbCamera cameraRaw = CameraServer.getInstance().startAutomaticCapture("Normal Camera", 0);
		cameraRaw.setResolution(IMG_WIDTH, IMG_HEIGHT);

		// Reflective detection camera
		UsbCamera cameraReflective = CameraServer.getInstance().startAutomaticCapture("Reflective Camera Raw", 1);
		cameraReflective.setResolution(IMG_WIDTH, IMG_HEIGHT);

		// Set settings for ideal camera settings for best reflective detection
		// cameraReflective.setExposureManual(10);
		// cameraReflective.setExposureHoldCurrent();
		cameraReflective.setBrightness(0);

		// Put the Reflection images to the dashboard
		CvSource cvOutput = CameraServer.getInstance().putVideo("Reflective Detection Output", IMG_WIDTH, IMG_HEIGHT);
		CvSink cvSink = CameraServer.getInstance().getVideo(cameraReflective);

		// Create the Mat object to work with the output of the detection code
		Mat mat = new Mat();

		// Create the vision detection thread and get the output of each frame
		VisionThread visionThread = new VisionThread(cameraReflective, new GripPipeline(), pipeline -> {
			// Bounding Rectangle
				Rect r = new Rect();

				// Draw a rectangle around the detection and get the center
				if (!pipeline.convexHullsOutput().isEmpty()) {
					// r =
					// Imgproc.boundingRect(pipeline.convexHullsOutput().get(0));
				synchronized (imgLock) {
					double total = 0;

					for (MatOfPoint p : pipeline.convexHullsOutput()) {
						r = Imgproc.boundingRect(p);
						total += r.x + (r.width / 2);
						centerX = total / pipeline.convexHullsOutput().size();
					}
					// centerX = r.x + (r.width / 2);
				}
			}

			// Get the processed frame and put it to the dashboard
			cvSink.grabFrame(mat);
			pipeline.process(mat);

			// Draw the rectangle and center of detection on the image
			Mat processedImage = pipeline.maskOutput();
			Imgproc.rectangle(processedImage, new Point(r.x, r.y), new Point(r.x + r.width, r.y + r.height), new Scalar(255, 0, 0), 1);
			Imgproc.drawContours(processedImage, pipeline.filterContoursOutput(), 0, new Scalar(0, 0, 255), 1);
			// cvOutput.putFrame(pipeline.maskOutput());
			cvOutput.putFrame(processedImage);
		});
		visionThread.start();
		}
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	@Override
	public void disabledInit() {

	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString code to get the auto name from the text box below the Gyro
	 *
	 * You can add additional auto modes by adding additional commands to the
	 * chooser code above (like the commented example) or additional comparisons
	 * to the switch structure below with additional strings & commands.
	 */
	@Override
	public void autonomousInit() {
		autonomousCommand = autonomousRoutinePicker.getSelected();
		autonomousLoopCounter = 0; // Set The Loop Counter = 0

		/*
		 * String autoSelected = SmartDashboard.getString("Auto Selector",
		 * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
		 * = new MyAutoCommand(); break; case "Default Auto": default:
		 * autonomousCommand = new ExampleCommand(); break; }
		 */

		// schedule the autonomous command (example)
		if (autonomousCommand != null)
			autonomousCommand.start();
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run(); // Run any called commands

		// 50 Autonomous Loops = 1 Second
		autonomousLoopCounter++;

	}

	@Override
	public void teleopInit() {

		// Stops Autonomous Command if not Already Done
		if (autonomousCommand != null)
			autonomousCommand.cancel();
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run(); // Run any called commands

		// ----- Drive Code -----

		// ~~ B-Button Drive ~~
		// B button pressed & Right Trigger is beyond Halfway pressed drive in
		// reverse
		if (RobotMap.xboxController.getRawButton(2) && RobotMap.xboxController.getRawAxis(3) > 0.5)
			RobotMap.robotDrive.tankDrive(1.0, 1.0);
		
		// Drive full Speed Strait Forward
		else if (RobotMap.xboxController.getRawButton(2)) // B button pressed
			RobotMap.robotDrive.tankDrive(-1.0, -1.0);

		// ~~ D-Pad Drive ~~
		// Drives Forward at Default D-Pad Drive Speed speed forward if the D-Pad is pressed Up ^
		else if (RobotMap.xboxController.getPOV() == 0)
			RobotMap.robotDrive.tankDrive(-DEFAULT_DPAD_SPEED, -DEFAULT_DPAD_SPEED);

		// Drives Forward at Default D-Pad Drive Speed speed forward and turns Left if the D-Pad is
		// pressed Up and Left <^
		else if (RobotMap.xboxController.getPOV() == 315)
			RobotMap.robotDrive.tankDrive(-DEFAULT_DPAD_SPEED / 2, -DEFAULT_DPAD_SPEED);

		// Drives Forward at Default D-Pad Drive Speed speed forward and turns Right if the D-Pad is
		// pressed Up and Right ^>
		else if (RobotMap.xboxController.getPOV() == 45)
			RobotMap.robotDrive.tankDrive(-DEFAULT_DPAD_SPEED, -DEFAULT_DPAD_SPEED / 2);

		// Turns Robot Right if the D-Pad is pressed Right >
		else if (RobotMap.xboxController.getPOV() == 90)
			RobotMap.robotDrive.tankDrive(-DEFAULT_DPAD_SPEED / 4, DEFAULT_DPAD_SPEED / 4);

		// Drives Forward at Default D-Pad Drive Speed speed backward if Down on the D-Pad is pressed
		// \/
		else if (RobotMap.xboxController.getPOV() == 180)
			RobotMap.robotDrive.tankDrive(DEFAULT_DPAD_SPEED, DEFAULT_DPAD_SPEED);

		// Drives Forward at Default D-Pad Drive Speed speed backward and turns Left if the D-Pad is
		// pressed Down and Left <\/
		else if (RobotMap.xboxController.getPOV() == 225)
			RobotMap.robotDrive.tankDrive(DEFAULT_DPAD_SPEED / 2, DEFAULT_DPAD_SPEED);

		// Drives Forward at Default D-Pad Drive Speed speed forward and turns Right if the D-Pad is
		// pressed Up and Right \/>
		else if (RobotMap.xboxController.getPOV() == 135)
			RobotMap.robotDrive.tankDrive(DEFAULT_DPAD_SPEED, DEFAULT_DPAD_SPEED / 2);

		// Turns Robot Left if the D-Pad is pressed Left <
		else if (RobotMap.xboxController.getPOV() == 270)
			RobotMap.robotDrive.tankDrive(DEFAULT_DPAD_SPEED / 4, -DEFAULT_DPAD_SPEED / 4);

		// ~~ Drive With Triggers ~~
		// Left Trigger controls left wheels, Right Trigger controls right
		// wheels
		// else if (RobotMap.xboxController.getRawAxis(2) != 0 ||
		// RobotMap.xboxController.getRawAxis(3) != 0)
		// If the A button is held down the robot drives in reverse
		// if (RobotMap.xboxController.getRawButton(1))
		// RobotMap.robotDrive.tankDrive(RobotMap.xboxController.getRawAxis(3),
		// RobotMap.xboxController.getRawAxis(2));
		// If The A Button isn't held, it then goes forward
		// else
		// RobotMap.robotDrive.tankDrive(-RobotMap.xboxController.getRawAxis(3),
		// -RobotMap.xboxController.getRawAxis(2));

		// ~~ Drive With Bumbers ~~
		// Turns Robot Slightly Left if Right bumper is pressed
		// if (RobotMap.xboxController.getRawButton(5))
		// RobotMap.robotDrive.drive(1, -.5);

		// Turns Robot Slightly Right if Right bumper is pressed
		// else if (RobotMap.xboxController.getRawButton(6))
		// RobotMap.robotDrive.drive(1, .5);

		// ~~ Joystick Drive ~~
		// Y Axis of Left joystick controls the Left wheels
		// Y Axis of Right joystick controls the Right wheels
		else
			RobotMap.robotDrive.tankDrive(RobotMap.xboxController.getRawAxis(5), RobotMap.xboxController.getRawAxis(1));

		// ----- Climber Code -----

		// Turns Robot Slightly Left if Right bumper is pressed
		if (RobotMap.xboxController.getRawButton(5))
			RobotMap.climberMotor.set(-1);

		// Turns Robot Slightly Right if Right bumper is pressed
		else if (RobotMap.xboxController.getRawButton(6))
			RobotMap.climberMotor.set(1);

		// Stop Climber if no bumper is pressed
		else
			RobotMap.climberMotor.stopMotor();

		// ----- Shooter Code -----

		if (RobotMap.ballShotSwitch.get()) {
			shooterAdditionalVelocityCounter = shooterAdditionalVelocity;
			shooterHoldSteadySpeed = getShooterHoldSteadySpeed() * shooterAdditionalVelocityMultiplier;
		}
		
		// Toggle Hold Steady Mode
		if (RobotMap.xboxController.getRawButton(3)) {
			isShooterHoldSteady = !isShooterHoldSteady;

			// Hold Steady Code
			if (isShooterHoldSteady) {
				shooterHoldSteadySpeed = getShooterHoldSteadySpeed();
			}
		}

		// Set the speed of the Shooter Motor
		// Y-Button held and Trigger pressed; Set hold speed to reverse trigger
		// speed
		if (RobotMap.xboxController.getRawAxis(2) != 0 && RobotMap.xboxController.getRawButton(4))
			RobotMap.shooterMotor.set(DEFAULT_MIN_SHOOTER_SPEED + RobotMap.xboxController.getRawAxis(3) * 0.45);
		// Trigger pressed; Set hold speed to forward trigger speed
		else if (RobotMap.xboxController.getRawAxis(2) != 0)
			RobotMap.shooterMotor.set(-DEFAULT_MIN_SHOOTER_SPEED - RobotMap.xboxController.getRawAxis(2) * 0.45);
		// X-Button pressed/Hold Steady Mode On; Hold Shooter Speed Constant
		else if (isShooterHoldSteady)
			RobotMap.shooterMotor.set(shooterHoldSteadySpeed);
		// Stop the motor
		else
			RobotMap.shooterMotor.set(0);
		
		if (shooterAdditionalVelocityCounter > 0) {
			RobotMap.shooterMotor.set(shooterHoldSteadySpeed);
		}
	}
	
	// Get the correct hold steady speed
	private double getShooterHoldSteadySpeed() {
		// Set the speed at which to hold the shooter motor at (%)

		// Y-Button held and Trigger pressed; Set hold speed to reverse
		// trigger speed
		if (RobotMap.xboxController.getRawAxis(2) != 0 && RobotMap.xboxController.getRawButton(4))
			return DEFAULT_MIN_SHOOTER_SPEED + RobotMap.xboxController.getRawAxis(3) * 0.45;
		// Y-Button held; Set hold speed to reverse default minimum
		// speed
		else if (RobotMap.xboxController.getRawButton(4))
			return DEFAULT_MIN_SHOOTER_SPEED;
		// Trigger pressed; Set hold speed to forward trigger speed
		else if (RobotMap.xboxController.getRawAxis(2) != 0)
			return DEFAULT_MIN_SHOOTER_SPEED + RobotMap.xboxController.getRawAxis(3) * 0.45;
		// Set hold speed to forward default minimum speed
		else
			return -DEFAULT_MIN_SHOOTER_SPEED;
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
		LiveWindow.run();
	}
}
