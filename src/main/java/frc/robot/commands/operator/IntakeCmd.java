package frc.robot.commands.operator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.operator.IntakeSubsystem;

public class IntakeCmd extends Command{

    private final IntakeSubsystem intakeSubsystem;
    private final double speed;

    public IntakeCmd(IntakeSubsystem intakeSubsystem, double speed){
        this.intakeSubsystem = intakeSubsystem;
        this.speed = speed;
        addRequirements(intakeSubsystem);
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      intakeSubsystem.setMotor(speed);
      //System.out.println("speed" + speed);
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      intakeSubsystem.setMotor(0);
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
  
      return false;
    }
}


