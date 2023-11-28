package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase{
    TalonSRX claw = new TalonSRX(11);
    
    
    private double avgCurrent = 0;
    private final double stuckCurrentThreshold = 15;
    
    public Claw() {
        claw.setInverted(true);
    }

    //intake the object
    public void intakeObject(){
        claw.set(ControlMode.PercentOutput, 0.45);
    }

    //intake the object at a faster speed for autonomous
    public void intakeObjectHard(){
        claw.set(ControlMode.PercentOutput, 0.6);

    }

    //release the object
    public void releaseObjectAuto(){
        claw.set(ControlMode.PercentOutput, -0.22);
    }

    public void releaseObject(){
        claw.set(ControlMode.PercentOutput, -0.15);
    }

    //stop the motor
    public void stop(){
        claw.set(ControlMode.PercentOutput, 0);
    }

    //check whether or not the claw is stuck
    public boolean isStuck() {
        //0.5 sec at stuckCurrentThreshold should be enough to trigger
        avgCurrent = .9 * avgCurrent + .1 * claw.getSupplyCurrent();
        System.out.println(claw.getSupplyCurrent());
        if(avgCurrent > stuckCurrentThreshold) {
            System.out.println("stuck");
            return true;
        }
        return false;
    }
}
