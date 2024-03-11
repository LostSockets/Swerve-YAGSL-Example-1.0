package frc.robot.subsystems.operator;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
//import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants;


public class ElevatorSubsystem extends SubsystemBase{

    private final CANSparkMax elevatorMotor1 = new CANSparkMax(Constants.OperatorConstants.ELEVATOR_MOTOR1_PORT, MotorType.kBrushless);
    private final CANSparkMax elevatorMotor2 = new CANSparkMax(Constants.OperatorConstants.ELEVATOR_MOTOR2_PORT, MotorType.kBrushless);
    private final RelativeEncoder elevatorEncoder = elevatorMotor1.getEncoder();

    public double getEncoderMeters() {
        return (((RelativeEncoder) elevatorEncoder).getPosition());
      }
    
    
    public ElevatorSubsystem(){

    }
    
    @Override
    public void periodic() {
        //SmartDashboard.putNumber("ArmPivotEncoder Value",  getEncoderMeters());
    }

    public void setMotor(double speed) {
        //SmartDashboard.putNumber("pivot speed", speed);
        elevatorMotor1.set(speed);
        elevatorMotor2.set(speed);
    }
   

}
