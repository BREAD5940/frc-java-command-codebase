package frc.robot;

public interface arcademap {


    public interface buttons {
    //Rocket: Cargo height

        public static final int level1Hatch = 1;
        public static final int level2Hatch = 2; 
        public static final int level3Hatch = 3;

    //Rocket: Cargo height

        public static final int level1Cargo = 4;
        public static final int level2Cargo = 5;
        public static final int level3Cargo = 6;

    //Wrist controls:
        //Intake controls

        public static final int intakeButton = 7;
        public static final int outtakeButton = 8;

        //Auto actions

        public static final int autoHatchPickUp = 11; 
        public static final int cargoCargoButton = 12;
    }

    public interface stick {
        public static final int elevatorAxis = 1;
        public static final int wristElbowAxis = 1;
    }
    
}