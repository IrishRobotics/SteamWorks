package org.usfirst.frc.team2606.robot.commands;

import org.usfirst.frc.team2606.robot.Robot;
import org.usfirst.frc.team2606.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

/**
 *  This Autonomous Command makes the robot drive forward for a set amount of time then stop
 */
public class AutonomousDriveForward extends Command {

	// Number of Loops To drive Forward
	// 50 Loops = 1 Second
	private final int NUM_DRIVE_LOOPS = 100;
	
    public AutonomousDriveForward() {
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if (Robot.autonomousLoopCounter < NUM_DRIVE_LOOPS) RobotMap.robotDrive.drive(.5, 0);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return !(Robot.autonomousLoopCounter < NUM_DRIVE_LOOPS);
    }

    // Called once after isFinished returns true
    protected void end() {
    	RobotMap.robotDrive.stopMotor();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
