package frc.robot.subsystems.operator;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
//import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants;


public class ShooterSubsystem extends SubsystemBase{

    private final CANSparkMax shooterMotor = new CANSparkMax(Constants.OperatorConstants.SHOOTER_MOTOR_PORT, MotorType.kBrushed);
    
    public ShooterSubsystem(){

    }
     @Override
    public void periodic() {
        //SmartDashboard.putNumber("ArmPivotEncoder Value",  getEncoderMeters());
    }

    public void setMotor(double speed) {
        //SmartDashboard.putNumber("pivot speed", speed);
        shooterMotor.set(speed);
    }
   

}
