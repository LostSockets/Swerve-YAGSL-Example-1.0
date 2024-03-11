package frc.robot.commands.operator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.operator.ElevatorSubsystem;

public class ElevatorPIDCmd extends Command{

    private final ElevatorSubsystem elevatorSubsystem;
    private final PIDController pidController;
    private final XboxController operatorXbox = new XboxController(OperatorConstants.JOYSTICK_OPERATOR);

    public ElevatorPIDCmd(ElevatorSubsystem elevatorSubsystem, double setpoint){
        this.elevatorSubsystem = elevatorSubsystem;
        this.pidController = new PIDController(Constants.OperatorPIDConstants.ELEVATOR_P,Constants.OperatorPIDConstants.ELEVATOR_I,Constants.OperatorPIDConstants.ELEVATOR_D);
        pidController.setSetpoint(setpoint);
        addRequirements(elevatorSubsystem);
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      double speed = pidController.calculate(elevatorSubsystem.getEncoderMeters());
      elevatorSubsystem.setMotor(speed);
      //System.out.println("speed" + speed);
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      elevatorSubsystem.setMotor(0);
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      if (Math.abs(operatorXbox.getRawAxis(Constants.OperatorConstants.ELEVATOR_AXIS)) > 0.05) {
        return true;
      } else {
        return false;
      }
      }
}


