package frc.robot.commands.operator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.operator.IndexerSubsystem;

public class IndexerCmd extends Command{

    private final IndexerSubsystem indexerSubsystem;
    private final double speed;

    public IndexerCmd(IndexerSubsystem indexerSubsystem, double speed){
        this.indexerSubsystem = indexerSubsystem;
        this.speed = speed;
        addRequirements(indexerSubsystem);
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      indexerSubsystem.setMotor(speed);
      //System.out.println("speed" + speed);
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      indexerSubsystem.setMotor(0);
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
  
      return false;
    }
}


