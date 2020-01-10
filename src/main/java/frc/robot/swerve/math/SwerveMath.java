package frc.robot.swerve.math;

import java.util.Arrays;
import java.util.List;

/**
 * This is the main class for the swerveDrive library.
 * The library handles the calculations required to drive a robot using SwerveDrive wheels. These wheels have two motors:
 * A drive motor that moves the robot and a turn motor that changes the wheel assembly's direction.
 *
 * MOTOR LAYOUT
 *
 * 					Front
 * 		Wheel 2 -------------- Wheel 1
 * 			|					|
 * 			|					|
 * 			|					|
 *	Left 	|					|   Right
 * 			|					|
 * 			|					|
 * 			|					|
 * 		Wheel 3 -------------- Wheel 4
 * 					Back
 *
 * The library supports two modes: Robot centric and Field centric. In Robot centric mode the robot turns relative to its
 * current position: 45 degrees to the right will turn the robot 45 degrees to the right (for example, if it is pointing
 * north before the turn, it will point north-east after the turn. In Field centric mode the robot turns to face the given
 * number of degrees relative to the firld's orientation: 0 means straight ahead down the field, 90 means to the right, etc.
 */
public class SwerveMath {
    // Robot dimensions. Units are of no importance. Required
    private final double lengthDef;
    private final double widthDef;
    private final double lengthPlus;
    private final double widthPlus;


    // The diagonal of the robot dimensions. Internal
    private final double diagonalDef;
    private final double diagonalPlus;

    // The scale factor to control robot maximum speed. Optional.
    private final double SCALE_SPEED = 1.00;

    // The "Centric" mode for the robot
    	private CentricMode centricMode = CentricMode.ROBOT;

    public void setModeField() {
		centricMode= CentricMode.FIELD;
	}
    /**
     * Constructor
     * @param widthDef the robot widthDef (units do not matter)
     * @param lengthDef the robot lengthDef (units do not matter)
     * @param widthPlus the robot widthPlus (units do not matter)
     * @param lengthPlus the robot lengthPlus (units do not matter)
     */
    
    public SwerveMath(double widthDef, double lengthDef, double widthPlus, double lengthPlus) {
        assert (widthDef > 0) : "Width has to be larger than 0";
        assert (lengthDef > 0) : "Length has to be larger than 0";
        assert (widthPlus > 0) : "Width has to be larger than 0";
        assert (lengthPlus > 0) : "Length has to be larger than 0";

        this.widthDef = widthDef;
        this.lengthDef = lengthDef;
        this.widthPlus = widthPlus;
        this.lengthPlus = lengthPlus;

        diagonalDef = Math.sqrt(Math.pow(this.lengthDef, 2) + Math.pow(this.widthDef, 2));
        diagonalPlus = Math.sqrt(Math.pow(this.lengthPlus, 2) + Math.pow(this.widthPlus, 2));
    }


    public CentricMode getCentricMode() {
        return centricMode;
    }


    public void setCentricMode(CentricMode centricMode) {
        this.centricMode = centricMode;
    }


    /**
     * move
     * Moves the robot based on 3 inputs - fwd (forward), str(strafe), and rcw(rotation clockwise)
     * Inputs are between -1 and 1, with 1 being full power, -1 being full reverse, and 0 being neutral.
     * The method uses gyro for field centric driving, if it is enabled.
     * @param fwd the forward power value range -1.0(back) - 1.0(fwd)
     * @param str the strafe power value range -1.0(left) - 1.0(right)
     * @param rcw the rotation power value range -1.0(ccw) - 1.0(cw)
     * @param gyroValue the value of the gyro input to be used by the calculation. Optional. Only used when the robot is in field-centric mode. Values are 0-360
     * @return List of wheel movement directives. The list indices correspond to the wheel numbering scheme as above, zero-based.
     */
    public List<SwerveDirective> move(double fwd, double str, double rcw, Double gyroValue) {

        if ((gyroValue == null) && centricMode.equals(CentricMode.FIELD)) {
            throw new IllegalStateException("Cannot use field centric mode without a Gyro value");
        }
        
        //Adjust for Gyro (if wanted)
        if (isFieldCentric()){
            //Convert the gyro angle (in degrees) to radians.
            double gyro = (gyroValue * Math.PI) / 180;

            double temp = fwd * Math.cos(gyro) + str * Math.sin(gyro);
            str = -fwd * Math.sin(gyro) + str * Math.cos(gyro);
            fwd = temp;
        }

        //These 8 variables are used in the swerve drive calculations.

        //Default
        double a_Def = str - rcw*(lengthDef / diagonalDef);
        double b_Def = str + rcw*(lengthDef / diagonalDef);
        double c_Def = fwd - rcw*(widthDef / diagonalDef);
        double d_Def = fwd + rcw*(widthDef / diagonalDef);
        //Plus
        double a_Plus = str - rcw*(lengthPlus / diagonalPlus);
        double b_Plus = str + rcw*(lengthPlus / diagonalPlus);
        double c_Plus = fwd - rcw*(widthPlus / diagonalPlus);
        double d_Plus = fwd + rcw*(widthPlus / diagonalPlus);


        //These are the equations for the wheel speed, for motors 1-4.
        
        double wsNW =  Math.sqrt(Math.pow(b_Def,2)+Math.pow(d_Def,2));
        double wsNE =  Math.sqrt(Math.pow(b_Def,2)+Math.pow(c_Def,2));
        double wsSE =  Math.sqrt(Math.pow(a_Def,2)+Math.pow(c_Def,2));
        double wsSW =  Math.sqrt(Math.pow(a_Def,2)+Math.pow(d_Def,2));
        
        double wsW =  Math.sqrt(Math.pow(d_Plus,2)+Math.pow((b_Plus/2 + a_Plus/2),2));
        double wsN =  Math.sqrt(Math.pow(b_Plus,2)+Math.pow((d_Plus/2 + c_Plus/2),2));
        double wsE =  Math.sqrt(Math.pow(c_Plus,2)+Math.pow((b_Plus/2 + a_Plus/2),2));
        double wsS =  Math.sqrt(Math.pow(a_Plus,2)+Math.pow((d_Plus/2 + c_Plus/2),2));

        //These are the equations for the wheel angle, for motors 1-4
        double waNW =  Math.atan2(b_Def,d_Def)*180/Math.PI;
        double waNE =  Math.atan2(b_Def,c_Def)*180/Math.PI;
        double waSE =  Math.atan2(a_Def,c_Def)*180/Math.PI;
        double waSW =  Math.atan2(a_Def,d_Def)*180/Math.PI;

        double waW =  Math.atan2((b_Plus/2 + a_Plus/2),d_Def)*180/Math.PI;
        double waN =  Math.atan2(b_Def,(d_Plus/2 + c_Plus/2))*180/Math.PI;
        double waE =  Math.atan2((b_Plus/2 + a_Plus/2),c_Def)*180/Math.PI;
        double waS =  Math.atan2(a_Def,(d_Plus/2 + c_Plus/2))*180/Math.PI;

        //This is to normalize the speed (if the largest speed is greater than 1, change accordingly).
        double max = wsNW;
        if(wsNE>max) max = wsNE;
        if(wsSE>max) max = wsSE;
        if(wsSW>max) max = wsSW;
        if(wsW>max) max = wsW;
        if(wsN>max) max = wsN;
        if(wsE>max) max = wsE;
        if(wsS>max) max = wsS;
        if(max>1){
            wsNW/=max;
            wsNE/=max;
            wsSE/=max;
            wsSW/=max;
            wsW/=max;
            wsN/=max;
            wsE/=max;
            wsS/=max;
        }

        //Wheel angle was in the range of -180 to 180. Now its -.5 to .5
        waNW/=360;
        waNE/=360;
        waSE/=360;
        waSW/=360;
        waW/=360;
        waN/=360;
        waE/=360;
        waS/=360;


        //Used to scale the movement speeds for testing (so you don't crash into walls)
        wsNW*=SCALE_SPEED;
        wsNE*=SCALE_SPEED;
        wsSE*=SCALE_SPEED;
        wsSW*=SCALE_SPEED;
        wsW*=SCALE_SPEED;
        wsN*=SCALE_SPEED;
        wsE*=SCALE_SPEED;
        wsS*=SCALE_SPEED;

        SwerveDirective directiveNW = new SwerveDirective(waNW, wsNW);
        SwerveDirective directiveNE = new SwerveDirective(waNE, wsNE);
        SwerveDirective directiveSE = new SwerveDirective(waSE, wsSE);
        SwerveDirective directiveSW = new SwerveDirective(waSW, wsSW);
        SwerveDirective directiveW = new SwerveDirective(waW, wsW);
        SwerveDirective directiveN = new SwerveDirective(waN, wsN);
        SwerveDirective directiveE = new SwerveDirective(waE, wsE);
        SwerveDirective directiveS = new SwerveDirective(waS, wsS);

        return Arrays.asList(directiveNW, directiveNE, directiveSE, directiveSW, directiveW, directiveN, directiveE, directiveS);
    }

    private boolean isFieldCentric() {
        return centricMode.equals(CentricMode.FIELD);
    }

}
