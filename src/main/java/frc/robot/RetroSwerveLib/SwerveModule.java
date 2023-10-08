package frc.robot.RetroSwerveLib;

// import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;

public class SwerveModule {

    // declare state variables
    private final CANSparkMax DRIVE_MOTOR;
    private final CANSparkMax DIRECTION_MOTOR;

    private SparkMaxPIDController mPID;
    // private WPI_CANCoder CAN_Coder;
    private RelativeEncoder encoder;
    
    public SwerveModule(int DRIVE_MOTOR, int DIRECTION_MOTOR, int CAN_Coder) {
        // initialize the motor controlers
        this.DRIVE_MOTOR = new CANSparkMax(DRIVE_MOTOR, MotorType.kBrushless);
        this.DIRECTION_MOTOR = new CANSparkMax(DIRECTION_MOTOR, MotorType.kBrushless);

        // initialize the encoders 
        // this.CAN_Coder = new WPI_CANCoder(CAN_Coder);
        this.encoder = this.DIRECTION_MOTOR.getEncoder();
        this.encoder.setPositionConversionFactor((2*Math.PI)/(Constants.ENCODER_COUNTS_PER_REV));

        // initalizer the direction pid
        this.mPID = this.DIRECTION_MOTOR.getPIDController();

        // set the pid values
        mPID.setP(0.75); // to be determined
        mPID.setOutputRange(-.5, .5); 
        mPID.setPositionPIDWrappingEnabled(true);
        mPID.setPositionPIDWrappingMaxInput(Constants.ENCODER_COUNTS_PER_REV/2);
        mPID.setPositionPIDWrappingMinInput(-(Constants.ENCODER_COUNTS_PER_REV/2));

        // seed the motors internal encoders with the absolute position of the cancoder
        updateEncoder();
    }


    //Controls the motors

    // set the speed of the drive motor
    public void setDriveMotor(double pow) {
        DRIVE_MOTOR.set(pow);
    }

    // set the angle of the direction motor
    public void setDirectionMotor(double Radians) {
        //double setPoint = Radians * (4.2/180); // this conversion will have to change based on the gear ratio of the swerve modules
        mPID.setReference(Radians/2, ControlType.kPosition);
    }

    //Direction motor encoder

    // sets the position of the direction encoder to be the same as the absolute value from the CAN_Coder
    public void updateEncoder() {
        //encoder.setPosition(CAN_Coder.getPosition()*(Math.PI/180));
        encoder.setPosition(0);
    }

    //Gets the encoder value
    public double getEncoder(){
        return encoder.getPosition();
    }
}
