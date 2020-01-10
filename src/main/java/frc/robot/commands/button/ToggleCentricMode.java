package frc.robot.commands.button;

import frc.robot.OI;
import frc.robot.subsystems.Drivetrain;
import frc.robot.swerve.math.CentricMode;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 * Switches the robot from field centric to robot centric, or vice versa.
 */
public class ToggleCentricMode extends Command {

	private Drivetrain drivetrain;
	private double startTime;

	public ToggleCentricMode(Drivetrain drivetrain) {
		this.drivetrain = drivetrain;
		requires(drivetrain);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		startTime = Timer.getFPGATimestamp();

		if (drivetrain.getCentricMode() == CentricMode.ROBOT) {
			drivetrain.setCentricMode(CentricMode.FIELD);
			OI.driveController.setRumble(RumbleType.kRightRumble, 1.0);
		} else if (drivetrain.getCentricMode() == CentricMode.FIELD) {
			drivetrain.setCentricMode(CentricMode.ROBOT);
			OI.driveController.setRumble(RumbleType.kLeftRumble, 1.0);
		}
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return Timer.getFPGATimestamp() >= startTime + 0.2;
	}

	// Called once after isFinished returns true
	protected void end() {
		OI.driveController.setRumble(RumbleType.kLeftRumble, 0.0);
		OI.driveController.setRumble(RumbleType.kRightRumble, 0.0);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		OI.driveController.setRumble(RumbleType.kLeftRumble, 0.0);
		OI.driveController.setRumble(RumbleType.kRightRumble, 0.0);
	}
}