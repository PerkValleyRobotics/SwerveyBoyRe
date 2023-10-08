package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RetroSwerveLib.*;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveDriveCMD extends CommandBase{
    private final SwerveSubsystem swerveSubsystem;
    private final DoubleSupplier getLeftX;
    private final DoubleSupplier getLeftY;
    private final DoubleSupplier getRightX;
    private final DoubleSupplier getRightY;

    public SwerveDriveCMD(SwerveSubsystem swerveSubsystem, DoubleSupplier getLeftX, DoubleSupplier getLeftY, DoubleSupplier getRightX, DoubleSupplier getRightY){
        this.swerveSubsystem = swerveSubsystem;
        this.getLeftX = getLeftX;
        this.getLeftY = getLeftY;
        this.getRightX = getRightX;
        this.getRightY = getRightY;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute(){
        //turn the left joystick position into a vector
        // double[] strafeVector = {getLeftX.getAsDouble(), getLeftY.getAsDouble()};

        //turn the right joystick posion into a vector 
        // double[] targetDirection = {getRightX.getAsDouble(), getRightY.getAsDouble()};

        //get the angle of the right joystick vector
        // double angle = SwerveMath.getAngle(targetDirection);

        //pass the desired strafe direction vector and the target angele(in radians) into the swerveSubsystem drive method
        // swerveSubsystem.drive(strafeVector, angle);

        //hear me out
        swerveSubsystem.drive(
            new double[] {getLeftX.getAsDouble(), getLeftY.getAsDouble()}, 
            SwerveMath.getAngle(new double[] {getRightX.getAsDouble(), getRightY.getAsDouble()})
        );
    }
}
