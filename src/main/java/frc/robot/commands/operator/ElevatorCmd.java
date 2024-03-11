package frc.robot.commands.operator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.operator.ElevatorSubsystem;

public class ElevatorCmd extends Command{

    private final ElevatorSubsystem elevatorSubsystem;
    private final double speed;

    public ElevatorCmd(ElevatorSubsystem elevatorSubsystem, double speed){
        this.elevatorSubsystem = elevatorSubsystem;
        this.speed = speed;
        addRequirements(elevatorSubsystem);
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
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
  
      return false;
    }
}


