package frc.robot;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

public class SwerveDrive {
    CANSparkMax drivingMotor;
    CANSparkMax turningMotor;

    RelativeEncoder drivingEncoder;
    AbsoluteEncoder turningEncoder;

    SparkMaxPIDController drivingPID;
    SparkMaxPIDController turningPID;

    double angleOffset;

    public SwerveDrive(int DrivingMotorCanID, int turningMOtorCanID, double angleOffset){
        drivingMotor = new CANSparkMax(DrivingMotorCanID, MotorType.kBrushless);
        turningMotor = new CANSparkMax(turningMOtorCanID, MotorType.kBrushless);
        this.angleOffset = angleOffset;

        drivingEncoder = drivingMotor.getEncoder();
        turningEncoder = turningMotor.getAbsoluteEncoder(Type.kDutyCycle);
        drivingPID = drivingMotor.getPIDController();
        turningPID = turningMotor.getPIDController();
        drivingPID.setFeedbackDevice(drivingEncoder);
        turningPID.setFeedbackDevice(turningEncoder);

        drivingEncoder.setPositionConversionFactor(Constants.Drive.positionConversionFactor);
        drivingEncoder.setVelocityConversionFactor(Constants.Drive.velocityConversionFactor);
        turningEncoder.setPositionConversionFactor(Constants.Turn.positionConversionFactor);
        turningEncoder.setVelocityConversionFactor(Constants.Turn.velocityConversionFactor);

        turningPID.setPositionPIDWrappingEnabled(true);
        turningPID.setPositionPIDWrappingMinInput(0);
        turningPID.setPositionPIDWrappingMaxInput(2 * Math.PI); 

        drivingPID.setP(Constants.Drive.p);
        drivingPID.setI(Constants.Drive.i);
        drivingPID.setD(Constants.Drive.d);
        drivingPID.setFF(Constants.Drive.ff);
        drivingPID.setOutputRange(-1, 1);

        turningPID.setP(Constants.Turn.p);
        turningPID.setI(Constants.Turn.i);
        turningPID.setD(Constants.Turn.d);
        turningPID.setFF(Constants.Turn.ff);
        turningPID.setOutputRange(-1, 1);

        drivingMotor.setIdleMode(IdleMode.kBrake);
        turningMotor.setIdleMode(IdleMode.kBrake);

        drivingMotor.burnFlash();
        turningMotor.burnFlash();
        
        drivingEncoder.setPosition(0);
    }

    public void setAngle(double angle){
        turningPID.setReference(angle + angleOffset, CANSparkMax.ControlType.kPosition);
    }

    public void setSpeed(double speed){
        double realSpeed = speed;
        if(angleOffset == 0 || angleOffset == Math.PI){
            realSpeed = realSpeed * -1;
        }
        drivingPID.setReference(realSpeed, CANSparkMax.ControlType.kVelocity);
    }
}
