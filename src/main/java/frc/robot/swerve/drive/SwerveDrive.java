package frc.robot.swerve.drive;

import frc.robot.swerve.math.CentricMode;
import frc.robot.swerve.math.SwerveDirective;
import frc.robot.swerve.math.SwerveMath;

import java.util.List;

/**
 * The main class for the SwerveDrive subsystem: This class handles all aspects of controlling the swerve drive.
 * Use this class in your program if you want the easiest way to integrate swerve drive into your robot.
 */
public class SwerveDrive {
    // Enclosures 1-4 are the drive/steer combos
    private SwerveEnclosure swerveEnclosureNW;
    private SwerveEnclosure swerveEnclosureNE;
    private SwerveEnclosure swerveEnclosureSE;
    private SwerveEnclosure swerveEnclosureSW;
    private SwerveEnclosure swerveEnclosureW;
    private SwerveEnclosure swerveEnclosureN;
    private SwerveEnclosure swerveEnclosureE;
    private SwerveEnclosure swerveEnclosureS;

    private final SwerveMath swerveMath;

    public SwerveDrive(SwerveEnclosure swerveEnclosureNW,
                       SwerveEnclosure swerveEnclosureNE,
                       SwerveEnclosure swerveEnclosureSE,
                       SwerveEnclosure swerveEnclosureSW,
                       SwerveEnclosure swerveEnclosureW,
                       SwerveEnclosure swerveEnclosureN,
                       SwerveEnclosure swerveEnclosureE,
                       SwerveEnclosure swerveEnclosureS,
                       double widthDef, double lengthDef, double widthPlus, double lengthPlus) {

        this.swerveEnclosureNW = swerveEnclosureNW;
        this.swerveEnclosureNE = swerveEnclosureNE;
        this.swerveEnclosureSE = swerveEnclosureSE;
        this.swerveEnclosureSW = swerveEnclosureSW;
        this.swerveEnclosureW = swerveEnclosureW;
        this.swerveEnclosureN = swerveEnclosureN;
        this.swerveEnclosureE = swerveEnclosureE;
        this.swerveEnclosureS = swerveEnclosureS;

        // instantiate the swerve library with a gyro provider using pigeon1
        swerveMath = new SwerveMath(widthDef, lengthDef, widthPlus, lengthPlus);

    }

    /**
     * move
     * Moves the robot based on 3 inputs - fwd (forward), str(strafe), and rcw(rotation clockwise)
     * Inputs are between -1 and 1, with 1 being full power, -1 being full reverse, and 0 being neutral.
     * The method uses gyro for field centric driving, if it is enabled.
     *
     * @param fwd
     * @param str
     * @param rcw
     * @param gyroValue the value of the gyro input to be used by the calculation. Optional. Only used when the robot is in field-centric mode.
     */
    public void move(double fwd, double str, double rcw, Double gyroValue) {
        // Get the move command calculated
        List<SwerveDirective> swerveDirectives = swerveMath.move(fwd, str, rcw, gyroValue);

        swerveEnclosureNW.move(swerveDirectives.get(0).getSpeed(), swerveDirectives.get(0).getAngle());
        swerveEnclosureNE.move(swerveDirectives.get(1).getSpeed(), swerveDirectives.get(1).getAngle());
        swerveEnclosureSE.move(swerveDirectives.get(2).getSpeed(), swerveDirectives.get(2).getAngle());
        swerveEnclosureSW.move(swerveDirectives.get(3).getSpeed(), swerveDirectives.get(3).getAngle());
        swerveEnclosureW.move(swerveDirectives.get(4).getSpeed(), swerveDirectives.get(4).getAngle());
        swerveEnclosureN.move(swerveDirectives.get(5).getSpeed(), swerveDirectives.get(5).getAngle());
        swerveEnclosureE.move(swerveDirectives.get(6).getSpeed(), swerveDirectives.get(6).getAngle());
        swerveEnclosureS.move(swerveDirectives.get(7).getSpeed(), swerveDirectives.get(7).getAngle());
    }

    /**
     * Stop the robot (set speed to 0)
     * @throws Exception 
     */
    public void stop() {
        swerveEnclosureNW.stop();
        swerveEnclosureNE.stop();
        swerveEnclosureSE.stop();
        swerveEnclosureSW.stop();
        swerveEnclosureW.stop();
        swerveEnclosureN.stop();
        swerveEnclosureE.stop();
        swerveEnclosureS.stop();
    }

    /**
     * Change the centric-mode of the robot (this can be done dynamically any time and will affect
     * the robot behavior from that point on)
     */
    public void setCentricMode(CentricMode centricMode) {
        this.swerveMath.setCentricMode(centricMode);
    }
    
    public void setModeField() {
		this.swerveMath.setModeField();
	}

}
