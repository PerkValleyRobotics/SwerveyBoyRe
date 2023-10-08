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

    private double[] speeds = new double[4];

    public void drive(double[] strafeVector, double angle){

        // calculate the appropriate radians per second to reach the desired angle
        omega = pidController.calculate(SwerveMath.getYawInRadians(), angle);

        // determine the sum of the strafe vecors with the turning vectors for each module to go to
        FRONT_RIGHT_VECTOR = SwerveMath.addVectors(SwerveMath.optimalAngle(FRONT_RIGHT_VECTOR, strafeVector),  SwerveMath.multByScalar(SwerveMath.findPerpendicular(Constants.FRONT_RIGHT_R), omega));
        FRONT_LEFT_VECTOR = SwerveMath.addVectors(SwerveMath.optimalAngle(FRONT_LEFT_VECTOR, strafeVector),  SwerveMath.multByScalar(SwerveMath.findPerpendicular(Constants.FRONT_LEFT_R), omega));
        BACK_RIGHT_VECTOR = SwerveMath.addVectors(SwerveMath.optimalAngle(BACK_RIGHT_VECTOR, strafeVector),  SwerveMath.multByScalar(SwerveMath.findPerpendicular(Constants.BACK_RIGHT_R), omega));
        BACK_LEFT_VECTOR = SwerveMath.addVectors(SwerveMath.optimalAngle(BACK_LEFT_VECTOR, strafeVector),  SwerveMath.multByScalar(SwerveMath.findPerpendicular(Constants.BACK_LEFT_R), omega));

        // determine the speed of each wheel from the magnitude of each vector
        speeds[0] = (SwerveMath.getMagnitude(FRONT_RIGHT_VECTOR))*SwerveMath.cosScaling(FRONT_RIGHT_VECTOR, new double[] {Math.cos(FRONT_RIGHT.getEncoder())*Math.abs(speeds[0]), Math.sin(FRONT_RIGHT.getEncoder())*Math.abs(speeds[0])});
        speeds[1] = (SwerveMath.getMagnitude(FRONT_LEFT_VECTOR))*SwerveMath.cosScaling(FRONT_LEFT_VECTOR, new double[] {Math.cos(FRONT_LEFT.getEncoder())*Math.abs(speeds[1]), Math.sin(FRONT_RIGHT.getEncoder())*Math.abs(speeds[1])});
        speeds[2] = (SwerveMath.getMagnitude(BACK_RIGHT_VECTOR))*SwerveMath.cosScaling(BACK_RIGHT_VECTOR, new double[] {Math.cos(BACK_RIGHT.getEncoder())*Math.abs(speeds[2]), Math.sin(FRONT_RIGHT.getEncoder())*Math.abs(speeds[2])});
        speeds[3] = (SwerveMath.getMagnitude(BACK_LEFT_VECTOR))*SwerveMath.cosScaling(BACK_LEFT_VECTOR, new double[] {Math.cos(BACK_LEFT.getEncoder())*Math.abs(speeds[3]), Math.sin(FRONT_RIGHT.getEncoder())*Math.abs(speeds[3])});

        // if the highest speed is greater than one devide all speeds by it to avoid giving the motors a value greater than one
        double highestSpeed = Math.abs(SwerveMath.getHighest(speeds));
        if(Math.abs(highestSpeed) > 1) speeds = SwerveMath.divideAll(speeds, highestSpeed);
        
        // set the wheel speeds 
        FRONT_RIGHT.setDriveMotor(speeds[0]/Constants.MAX_SPEED_DIVISOR);
        FRONT_LEFT.setDriveMotor(speeds[1]/Constants.MAX_SPEED_DIVISOR);
        BACK_RIGHT.setDriveMotor(speeds[2]/Constants.MAX_SPEED_DIVISOR);
        BACK_LEFT.setDriveMotor(speeds[3]/Constants.MAX_SPEED_DIVISOR);

        // set the wheel angles and subtract from them the current robot yaw in order to maintain field centric driving
        FRONT_RIGHT.setDirectionMotor(SwerveMath.getAngle(FRONT_RIGHT_VECTOR)); // SwerveMath.offSet(SwerveMath.getAngle(FRONT_RIGHT_VECTOR)));
        FRONT_LEFT.setDirectionMotor(SwerveMath.getAngle(FRONT_LEFT_VECTOR)); //SwerveMath.offSet(SwerveMath.getAngle(FRONT_LEFT_VECTOR)));
        BACK_RIGHT.setDirectionMotor(SwerveMath.getAngle(BACK_RIGHT_VECTOR)); //SwerveMath.offSet(SwerveMath.getAngle(BACK_RIGHT_VECTOR)));
        BACK_LEFT.setDirectionMotor(SwerveMath.getAngle(BACK_LEFT_VECTOR)); //SwerveMath.offSet(SwerveMath.getAngle(BACK_RIGHT_VECTOR)));


        //Smart dashboard for testing
        SmartDashboard.putNumber("FR Speed", speeds[0]);
        SmartDashboard.putNumber("FL Speed", speeds[1]);
        SmartDashboard.putNumber("BR Speed", speeds[2]);
        SmartDashboard.putNumber("BL Speed", speeds[3]);

        SmartDashboard.putNumber("FR Angle", SwerveMath.offSet(SwerveMath.getAngle(FRONT_RIGHT_VECTOR)));
        SmartDashboard.putNumber("FL Angle", SwerveMath.offSet(SwerveMath.getAngle(FRONT_LEFT_VECTOR)));
        SmartDashboard.putNumber("BR Angle", SwerveMath.offSet(SwerveMath.getAngle(BACK_RIGHT_VECTOR)));
        SmartDashboard.putNumber("BL Angle", SwerveMath.offSet(SwerveMath.getAngle(BACK_LEFT_VECTOR)));

        SmartDashboard.putNumber("Angle", SwerveMath.getYawInRadians());
    }
}
