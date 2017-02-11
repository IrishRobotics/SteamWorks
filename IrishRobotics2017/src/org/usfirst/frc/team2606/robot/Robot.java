package org.usfirst.frc.team2606.robot;

import java.lang.reflect.Array;
import java.util.ArrayList;

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
	
	// Size of camera image
	private static int IMG_WIDTH;
	private static int IMG_HEIGHT;
	
	// Center of Image & Thread Lock object to used when obtaining the image
	private double centerX = 0.0;
	private final Object imgLock = new Object();

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
		
		// Vision Code
		IMG_WIDTH = 640;
		IMG_HEIGHT = 480;
		
		// Raw unchanged camera
		UsbCamera cameraRaw = CameraServer.getInstance().startAutomaticCapture("Normal Camera", 0);
    	cameraRaw.setResolution(IMG_WIDTH, IMG_HEIGHT);
		
    	// Reflective detection camera 
		UsbCamera cameraReflective = CameraServer.getInstance().startAutomaticCapture("Reflective Camera Raw", 1);
    	cameraReflective.setResolution(IMG_WIDTH, IMG_HEIGHT);
    	
    	// Set settings for ideal camera settings for best reflective detection
    	//cameraReflective.setExposureManual(10);
    	//cameraReflective.setExposureHoldCurrent();
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
                //r = Imgproc.boundingRect(pipeline.convexHullsOutput().get(0));
                synchronized (imgLock) {
                    double total = 0;
                    Mat m = new MatOfPoint(m)
                	
                	for(MatOfPoint p : pipeline.convexHullsOutput()) {
                		r = r | Imgproc.boundingRect(p);
                		total += r.x + (r.width / 2);
                		centerX = total / pipeline.convexHullsOutput().size();
                	}
                	//centerX = r.x + (r.width / 2);
               }
            }
    		
    		// Get the processed frame and put it to the dashboard
    		cvSink.grabFrame(mat);
    		pipeline.process(mat);
    		
    		// Draw the rectangle and center of detection on the image
    		Mat processedImage = pipeline.maskOutput();
    		Imgproc.rectangle(processedImage, new Point(r.x, r.y), new Point(r.x + r.width, r.y + r.height), new Scalar(255, 0, 0), 1);
    		Imgproc.drawContours(processedImage, pipeline.filterContoursOutput(), 0, new Scalar(0, 0, 255), 1);
    		//cvOutput.putFrame(pipeline.maskOutput());
    		cvOutput.putFrame(processedImage);
    	});
    	visionThread.start();
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

		// Drive Code

		// B button pressed & Right Trigger is beyond Halfway pressed drive in reverse
		if (RobotMap.xboxController.getRawButton(2)
				&& RobotMap.xboxController.getRawAxis(3) > 0.5)
			RobotMap.robotDrive.tankDrive(-1, -1);

		// Drive full Speed Strait Forward
		else if (RobotMap.xboxController.getRawButton(2)) // B button pressed
			RobotMap.robotDrive.tankDrive(1, 1);

		// Drives Forward at full speed forward if Up on the D-Pad is pressed
		else if (RobotMap.xboxController.getPOV() == 0)
			RobotMap.robotDrive.tankDrive(1, 1);

		// Drives Forward at full speed backward if Down on the D-Pad is pressed
		else if (RobotMap.xboxController.getPOV() == 180)
			RobotMap.robotDrive.tankDrive(-1, -1);

		// Drive With Triggers
		// Left Trigger controls left wheels, Right Trigger controls right wheels
		else if (RobotMap.xboxController.getRawAxis(2) != 0 || RobotMap.xboxController.getRawAxis(3) != 0)
			// If the A button is held down the robot drives in reverse
			if (RobotMap.xboxController.getRawButton(1)) RobotMap.robotDrive.tankDrive(RobotMap.xboxController.getRawAxis(3), RobotMap.xboxController.getRawAxis(2));
			// If The A Button isn't held, it then goes forward
			else RobotMap.robotDrive.tankDrive(-RobotMap.xboxController.getRawAxis(3), -RobotMap.xboxController.getRawAxis(2));
		
		// Y Axis of Left joystick controls the Left wheels
		// Y Axis of Right joystick controls the Right wheels
		else
			RobotMap.robotDrive.tankDrive(
					RobotMap.xboxController.getRawAxis(5),
					RobotMap.xboxController.getRawAxis(1));

		// Turns Robot Slightly Left if Right bumper is pressed
		//if (RobotMap.xboxController.getRawButton(5))
			//RobotMap.robotDrive.drive(1, -.5);

		// Turns Robot Slightly Right if Right bumper is pressed
		//else if (RobotMap.xboxController.getRawButton(6))
			//RobotMap.robotDrive.drive(1, .5);

		
		// Climber Code
		
		// Turns Robot Slightly Left if Right bumper is pressed
				if (RobotMap.xboxController.getRawButton(5))
					RobotMap.climberMotor.set(-1);

				// Turns Robot Slightly Right if Right bumper is pressed
				else if (RobotMap.xboxController.getRawButton(6))
					RobotMap.climberMotor.set(1);
				
				// Stop Climber if no bumper is pressed
				else RobotMap.climberMotor.stopMotor();
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
		LiveWindow.run();
	}
}
