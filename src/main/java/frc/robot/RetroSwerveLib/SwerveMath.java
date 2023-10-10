package frc.robot.RetroSwerveLib;

import com.kauailabs.navx.frc.AHRS;

public class SwerveMath {
    //Vector math

    //Adds two vectors
    public static double[] addVectors(double[] vector_1, double[] vector_2){
        return new double[] {vector_1[0] + vector_2[0], vector_1[1] + vector_2[1]};
    }

    //Finds the dotproduct of two vectors
    public static double dotProduct(double[] vectorI, double[] vectorII) {
        return vectorI[0]*vectorII[0] + vectorI[1]*vectorII[1];
    }

    //Gets the magnitude of a vector
    public static double getMagnitude(double[] vector){
        return Math.sqrt(Math.pow(vector[0], 2) + Math.pow(vector[1], 2));
    }

    //Finds the perpindicular vector to another vector
    public static double[] findPerpendicular(double[] vector){
        return new double[] {vector[1], -1 * vector[0]};
    }

    //Value scaling

    //Multiplies by scalar, or vector unit
    public static double[] multByScalar(double[] vector, double scalar){
        return new double[] {vector[0] * scalar, vector[1] * scalar};
    }

    //Scales the speed values of the motors so they do not fight eachother
    public static double cosScaling(double[] targetVector, double[] currentVector) {
        double magnitudes = getMagnitude(targetVector)*getMagnitude(currentVector);
        if (magnitudes != 0) return (dotProduct(targetVector, currentVector))/magnitudes;
        return 1;
    }

    //Puts a vector's values in the range of -1 to 1
    public static double[] createUnitVector(double[] vector, double magnitude){
        return new double[] {vector[0] / magnitude, vector[1] / magnitude};
    } 

    //Divides all values in a list
    public static double[] divideAll(double[] list, double d){
        double[] newList = new double[list.length];
        for(int i = 0; i < list.length; i++){
            newList[i] = list[i] / d;
        }
        return newList;
    }

    //Getter methods

    //Gets the angle of a vector
    public static double getAngle(double[] vector){
        return Math.atan2(vector[1], vector[0]);
    }

    //Gets the highest value in a list
    public static double getHighest(double[] list){
        double highest = 0;
        for(int i = 0; i < list.length; i++){
            if(Math.abs(list[i]) > highest){
                highest = Math.abs(list[i]);
            }
        }
        return highest;
    }

    //Optimization methods

    //Finds optimal vector to move to
    public static double[] optimalVector(double[] current, double[] target) {
        if (dotProduct(current, new double[] {target[0]*-1, target[1]*-1}) > dotProduct(current, target)) return new double[] {target[0]*-1, target[1]*-1};
        return target;
    }

    //Field oriented

    //NavX
    private final static AHRS navx = new AHRS();

    // return the current yaw according to the navx in radians rather than degrees 
    public static double getYawInRadians(){
        return (Math.round((navx.getYaw()%180 * (Math.PI/180))*10)/10);
    }

    //Gives the offset angle for field oriented drive
    public static double offSet(double targetAngle) {
        return (targetAngle - getYawInRadians());
    }

    //Controller deadband

    //If joystiocks do not center at 0,0 makes it go to 0,0
    public static double deadBand(double deadZone, double num) {
        if (Math.abs(num) < deadZone) return 0;
        return Math.floor(num*100)/100;
    }
}
