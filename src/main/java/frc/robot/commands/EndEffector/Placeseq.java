package frc.robot.commands.EndEffector;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Placeseq extends SequentialCommandGroup {

    public Placeseq() {

addCommands(
    new PlaceL4(),
    new PlaceL3(),
    new PlaceL2(),
    new PlaceL1()
    
);
    }
    


    
}
