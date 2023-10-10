package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RetroSwerveLib.*;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveSubsystem extends SubsystemBase{

    // declare state variables
    private final SwerveModule FRONT_RIGHT;
    private final SwerveModule FRONT_LEFT;
    private final SwerveModule BACK_RIGHT;
    private final SwerveModule BACK_LEFT;

    private final PIDController pidController;

    public SwerveSubsystem() {

        // initialize the swerve modules 
        this.FRONT_RIGHT = new SwerveModule(Constants.FRONT_RIGHT_DRIVE, Constants.FRONT_RIGHT_DIRECTION, Constants.FRONT_RIGHT_CANCODER);
        this.FRONT_LEFT = new SwerveModule(Constants.FRONT_LEFT_DRIVE, Constants.FRONT_LEFT_DIRECTION, Constants.FRONT_LEFT_CANCODER);
        this.BACK_RIGHT = new SwerveModule(Constants.BACK_RIGHT_DRIVE, Constants.BACK_RIGHT_DIRECTION, Constants.BACK_RIGHT_CANCODER);
        this.BACK_LEFT = new SwerveModule(Constants.BACK_LEFT_DRIVE, Constants.BACK_LEFT_DIRECTION, Constants.BACK_LEFT_CANCODER);

        // initialize the PID controller for the turn speed calculation
        pidController = new PIDController(0.5, 0, 0); // correct constant values need to be determined 
    }

    //Drive
    private double omega;

    private double[] FRONT_RIGHT_VECTOR = new double[2];
    private double[] FRONT_LEFT_VECTOR = new double[2];
    private double[] BACK_RIGHT_VECTOR = new double[2];
    private double[] BACK_LEFT_VECTOR = new double[2];

    private double[][] currentVectors = new double[4][2];
    private double[] speeds = new double[4];

    public void drive(double[] strafeVector, double angle){

        // calculate the appropriate radians per second to reach the desired angle
        omega = pidController.calculate(SwerveMath.getYawInRadians(), angle);

        /*
         * The vectors being set are the new target vectors for each swerve module
         * 
         * strafeVector is the target direction for the robot to move in
         * 
         * The perpindicular vector to the vectors representing the distance of each swerve module's distance from the center of the frame 
         * are multiplied by omega; omega is the scalar unit representing rotational velocity
         * 
         * After above mentioned calculations, the two resulting vectors are added together for each module in order to get the new target vectors
         */
        FRONT_RIGHT_VECTOR = SwerveMath.addVectors(strafeVector, SwerveMath.multByScalar(SwerveMath.findPerpendicular(Constants.FRONT_RIGHT_R), omega));
        FRONT_LEFT_VECTOR = SwerveMath.addVectors(strafeVector, SwerveMath.multByScalar(SwerveMath.findPerpendicular(Constants.FRONT_LEFT_R), omega));
        BACK_RIGHT_VECTOR = SwerveMath.addVectors(strafeVector, SwerveMath.multByScalar(SwerveMath.findPerpendicular(Constants.BACK_RIGHT_R), omega));
        BACK_LEFT_VECTOR = SwerveMath.addVectors(strafeVector, SwerveMath.multByScalar(SwerveMath.findPerpendicular(Constants.BACK_LEFT_R), omega));

        /*
         * currentVectors[0] - front right module; currentVectors[1] - front left module;  currentVectors[2] - back right module; currentVectors[3] - back left module;
         * 
         * The built-in encoder of each direction motor is used to get the current angle of each module
         * Cosine and sine of each module's angle is then multiplied by the previous set speed to get the current vector of each module
         */
        currentVectors[0] = new double[] {Math.cos(FRONT_RIGHT.getEncoder())*Math.abs(speeds[0]), Math.sin(FRONT_RIGHT.getEncoder())*Math.abs(speeds[0])};
        currentVectors[1] = new double[] {Math.cos(FRONT_LEFT.getEncoder())*Math.abs(speeds[1]), Math.sin(FRONT_LEFT.getEncoder())*Math.abs(speeds[1])};
        currentVectors[2] = new double[] {Math.cos(BACK_RIGHT.getEncoder())*Math.abs(speeds[2]), Math.sin(BACK_RIGHT.getEncoder())*Math.abs(speeds[2])};
        currentVectors[3] = new double[] {Math.cos(BACK_LEFT.getEncoder())*Math.abs(speeds[3]), Math.sin(BACK_LEFT.getEncoder())*Math.abs(speeds[3])};

        /* 
         * speed[0] - front right module; speed[1] - front left module;  speed[2] - back right module; speed[3] - back left module;
         * 
         * The magnitude of the target vector is multiplied by the difference in angle between the target vector and the current vector of a module
        */
        speeds[0] = (SwerveMath.getMagnitude(FRONT_RIGHT_VECTOR))*SwerveMath.cosScaling(FRONT_RIGHT_VECTOR, currentVectors[0]);
        speeds[1] = (SwerveMath.getMagnitude(FRONT_LEFT_VECTOR))*SwerveMath.cosScaling(FRONT_LEFT_VECTOR, currentVectors[1]);
        speeds[2] = (SwerveMath.getMagnitude(BACK_RIGHT_VECTOR))*SwerveMath.cosScaling(BACK_RIGHT_VECTOR, currentVectors[2]);
        speeds[3] = (SwerveMath.getMagnitude(BACK_LEFT_VECTOR))*SwerveMath.cosScaling(BACK_LEFT_VECTOR, currentVectors[3]);

        // Scales the speeds so that each motor is sent a value no greater than 1 nor less than -1
        double highestSpeed = Math.abs(SwerveMath.getHighest(speeds));
        if(Math.abs(highestSpeed) > 1) speeds = SwerveMath.divideAll(speeds, highestSpeed);
        
        // sets the wheel speeds to the previously calculated speeds
        FRONT_RIGHT.setDriveMotor(speeds[0]/Constants.MAX_SPEED_DIVISOR);
        FRONT_LEFT.setDriveMotor(speeds[1]/Constants.MAX_SPEED_DIVISOR);
        BACK_RIGHT.setDriveMotor(speeds[2]/Constants.MAX_SPEED_DIVISOR);
        BACK_LEFT.setDriveMotor(speeds[3]/Constants.MAX_SPEED_DIVISOR);

        /*
         * Sets the angles for each mosule's direction motor
         * 
         * A check is done to see if the current vector or the current opposite of the current vector is closest to the target varget
         * Ensuring the module never has to turn more than 90 degrees to reach the target position
         * 
         * An offset is then applied based on the angle of an onboard gyroscope in order to maintain field centric driving
         */
        FRONT_RIGHT.setDirectionMotor(SwerveMath.getAngle(SwerveMath.optimalVector(currentVectors[0], FRONT_RIGHT_VECTOR))); 
        FRONT_LEFT.setDirectionMotor(SwerveMath.getAngle(SwerveMath.optimalVector(currentVectors[1], FRONT_LEFT_VECTOR))); 
        BACK_RIGHT.setDirectionMotor(SwerveMath.getAngle(SwerveMath.optimalVector(currentVectors[2], BACK_RIGHT_VECTOR))); 
        BACK_LEFT.setDirectionMotor(SwerveMath.getAngle(SwerveMath.optimalVector(currentVectors[3], BACK_LEFT_VECTOR))); 

        //Smart dashboard for testing
        SmartDashboard.putNumber("FR Speed", speeds[0]);
        SmartDashboard.putNumber("FL Speed", speeds[1]);
        SmartDashboard.putNumber("BR Speed", speeds[2]);
        SmartDashboard.putNumber("BL Speed", speeds[3]);

        SmartDashboard.putNumber("FR Angle", SwerveMath.getAngle(SwerveMath.optimalVector(currentVectors[0], FRONT_RIGHT_VECTOR)));
        SmartDashboard.putNumber("FL Angle", SwerveMath.getAngle(SwerveMath.optimalVector(currentVectors[1], FRONT_LEFT_VECTOR)));
        SmartDashboard.putNumber("BR Angle", SwerveMath.getAngle(SwerveMath.optimalVector(currentVectors[2], BACK_RIGHT_VECTOR)));
        SmartDashboard.putNumber("BL Angle", SwerveMath.getAngle(SwerveMath.optimalVector(currentVectors[3], BACK_LEFT_VECTOR)));

        SmartDashboard.putNumber("Angle", SwerveMath.getYawInRadians());
    }
}
