package frc.robot;

public class Constants {

    public static class IO{
        public static int frontLeftDrive = 3;
        public static int frontLeftTurn = 4;

        public static int frontRightDrive = 1;
        public static int frontRightTurn = 2;
        
        public static int backLeftDrive = 7;
        public static int backLeftTurn = 8;

        public static int backRightDrive = 5;
        public static int backRightTurn = 6;

        public static int controller = 0;
    }

    public static class Drive{

        public static double positionConversionFactor = PhysicalParams.wheelDiameters * Math.PI / PhysicalParams.drivingMotorReduction;
        public static double velocityConversionFactor = positionConversionFactor / 60.0;
        
        public static double p = 0.4;
        public static double i = 0;
        public static double d = 0;
        public static double ff = 1/PhysicalParams.drivingFreeSpeedRPS;

    }

    public static class Turn{

        public static double positionConversionFactor = 2 * Math.PI;
        public static double velocityConversionFactor = 2 * Math.PI/60;

        public static double p = 0.7;
        public static double i = 0;
        public static double d = 0;
        public static double ff = 0;

        public static double[] angleOffsets = {0, Math.PI/2, -Math.PI/2, Math.PI};
        //frontRight, frontLeft, backRight, backLeft

    }

    public static class PhysicalParams{

        //in meters
        public static double freeSpeedRPM = 5676;
        public static double freeSpeedRPS = freeSpeedRPM/60;
        public static double wheelDiameters = 0.0762;
        public static double wheelCircumference = wheelDiameters * Math.PI;
        public static double drivingMotorPinionTeeth = 13;
        public static double bevelGear = 45;
        public static double firstStageSpurGear = 22;
        public static double bevelPinion = 15;
        public static double drivingMotorReduction = (bevelGear * firstStageSpurGear)/(drivingMotorPinionTeeth*bevelPinion);
        public static double drivingFreeSpeedRPS = (freeSpeedRPS * wheelCircumference)/drivingMotorReduction;

    }
}
