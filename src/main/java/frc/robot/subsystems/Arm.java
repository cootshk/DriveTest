package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
    //Potentiometer - min-val: 1 max-val: 100
    AnalogPotentiometer shoulderPot = new AnalogPotentiometer(1, 100, 0);
    AnalogPotentiometer elbowPot = new AnalogPotentiometer(2, 100, 0);
    //PID controller for arm
    PIDController pid = new PIDController(0.02, 0.0002, 0);
    //Spark Max of shoulder and elbow
    CANSparkMax shoulder = new CANSparkMax(15, MotorType.kBrushless);
    CANSparkMax elbow = new CANSparkMax(17, MotorType.kBrushless);

    RelativeEncoder shoulderEncoder;
    RelativeEncoder elbowEncoder;

   
    public double shoulderSetpoint=ArmConstants.SHOULDER_BASE_ANGLE;
    public double elbowSetpoint = ArmConstants.ELBOW_BASE_ANGLE;

    public Arm() {
        shoulderEncoder = shoulder.getEncoder();
        elbowEncoder = elbow.getEncoder();

        shoulder.setIdleMode(IdleMode.kBrake);
        elbow.setIdleMode(IdleMode.kBrake);

        shoulder.burnFlash();
        elbow.burnFlash();

        shoulderEncoder.setPosition(0);
        elbowEncoder.setPosition(0);
    }
    //Go to base position with the elbow
    public void setElbowBase(){
        elbowSetpoint=ArmConstants.ELBOW_BASE_ANGLE;
    }
    //Go to base position with the shoulder
    public void setShoulderBase(){
        shoulderSetpoint=ArmConstants.SHOULDER_BASE_ANGLE;
    }
    
    //Go to floor for intake
    public void moveToFloor() {
        shoulderSetpoint =  ArmConstants.SHOULDER_FLOOR_ANGLE;
        elbowSetpoint = ArmConstants.ELBOW_FLOOR_ANGLE;
    }

    //low platform for scoring cube
    public void lowPlatformCube() {
        shoulderSetpoint=ArmConstants.SHOULDER_LOW_PLATFORM_ANGLE;
        elbowSetpoint=ArmConstants.ELBOW_LOW_PLATFORM_ANGLE;
    }

    //high platform for scoring cube
    public void highPlatformCube() {
        shoulderSetpoint=ArmConstants.NEW_SHOULDER_HIGH_PLATFORM_ANGLE;
        elbowSetpoint=ArmConstants.NEW_ELBOW_HIGH_PLATFORM_ANGLE;
    }

    //autonomous cube angle
    public void quickCubeAngle(){
        shoulderSetpoint=ArmConstants.SHOULDER_QUICK_CUBE_ANGLE;
        elbowSetpoint=ArmConstants.ELBOW_QUICK_CUBE_ANGLE;
    }

    //low pole for scoring cones
    public void lowPoleCone() {
        shoulderSetpoint=ArmConstants.SHOULDER_LOW_POLE_ANGLE;
        elbowSetpoint=ArmConstants.ELBOW_LOW_POLE_ANGLE;
    }

    //high pole for scoring cones
    public void highPoleCone() {
        shoulderSetpoint=ArmConstants.SHOULDER_HIGH_POLE_ANGLE;
        elbowSetpoint=ArmConstants.ELBOW_HIGH_POLE_ANGLE;
    }

    //height to get substation
    public void substation() {
        shoulderSetpoint=ArmConstants.SHOULDER_SUBSTATION_ANGLE;
        elbowSetpoint=ArmConstants.ELBOW_SUBSTATION_ANGLE;
    }

    //move shoulder to output
    public void setShoulderOutput(double output) {
        output = MathUtil.clamp(output, -ArmConstants.ARM_MAX_SPEED, ArmConstants.ARM_MAX_SPEED);
        shoulder.set(output);
    }

    //move elbow to output
    public void setElbowOutput(double output) {
        output = MathUtil.clamp(output, -ArmConstants.ARM_MAX_SPEED, ArmConstants.ARM_MAX_SPEED);
        elbow.set(output);
    }

    //
    public void setSetpoints() {
        shoulderSetpoint = shoulderPotentiometerToDegrees(shoulderPot.get());
        elbowSetpoint = elbowPotentiometerToDegrees(elbowPot.get());
    }

    //go to location with val1 for shoulder and val2 for elbow
    public void goToLocation(double val1, double val2) {
        if (shoulderPot.get() > 90 || elbowPot.get() > 90) {
            stop();
            System.out.println("Potentiometer overload!");
            return;
        }
        double shoulderAngle = shoulderPotentiometerToDegrees(shoulderPot.get());
        double elbowAngle = elbowPotentiometerToDegrees(elbowPot.get());

        double shoulderOutput = pid.calculate(shoulderAngle, val1);
        double elbowOutput = pid.calculate(elbowAngle, val2);
      

        shoulderOutput = MathUtil.clamp(shoulderOutput, -ArmConstants.ARM_MAX_SPEED, ArmConstants.ARM_MAX_SPEED);
        elbowOutput = MathUtil.clamp(elbowOutput, -ArmConstants.ARM_MAX_SPEED, ArmConstants.ARM_MAX_SPEED);

       
            elbow.set(-elbowOutput);

        shoulder.set(shoulderOutput);

        

    }

    //stop moving the arm
    public void stop() {
        shoulder.set(0.001);
        elbow.set(0.001);

    }

    //if the arm has reached it's desired location
    public boolean reached(){
        return Math.abs(shoulderPotentiometerToDegrees(shoulderPot.get())-shoulderSetpoint)<5&&Math.abs(elbowPotentiometerToDegrees(elbowPot.get())-elbowSetpoint)<6;
    }

    //Print potentiometer values for debugging
    public void logPotentiometerValues() {
        System.out.println("Arm 1 Degrees: " + shoulderPotentiometerToDegrees(shoulderPot.get()));
        System.out.println("Arm 2 Degrees: " + elbowPotentiometerToDegrees(elbowPot.get()));
        /*System.out.println("Arm 1 Degrees: " + shoulderPot.get());
        System.out.println("Arm 2 Degrees: " + elbowPot.get());*/
    }

    //turn potentiometer values to degrees
    private double shoulderPotentiometerToDegrees(double potValue) {
        return (potValue - 73.13642396192131) * -6.480209440369113;
    }

    //turn potentiometer values to degrees
    private double elbowPotentiometerToDegrees(double potValue) {
        return (potValue - 59.02459090889985) * 6.480209440369113;
    }

    private double getArmExtensionLength(){
        return (24.1253) * Math.cos(Math.toRadians(shoulderPotentiometerToDegrees(shoulderPot.get()))) + (38.237) * Math.cos(Math.toRadians(shoulderPotentiometerToDegrees(shoulderPot.get())) + Math.toRadians(elbowPotentiometerToDegrees(elbowPot.get()))) - 11;
    }
}
