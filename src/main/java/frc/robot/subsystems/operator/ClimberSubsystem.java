package frc.robot.subsystems.operator;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
//import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants;


public class ClimberSubsystem extends SubsystemBase{

    private final CANSparkMax climberMotor = new CANSparkMax(Constants.OperatorConstants.CLIMBER_MOTOR_PORT, MotorType.kBrushless);
    
    public ClimberSubsystem(){

    }
     @Override
    public void periodic() {
        //SmartDashboard.putNumber("ArmPivotEncoder Value",  getEncoderMeters());
    }

    public void setMotor(double speed) {
        //SmartDashboard.putNumber("pivot speed", speed);
        climberMotor.set(speed);
    }
   

}
