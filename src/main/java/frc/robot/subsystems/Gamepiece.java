package frc.robot.subsystems;

public enum Gamepiece {
    CONE("CONE"), 
    CUBE("CUBE");
    private final String name;

    private Gamepiece(String name) {
        this.name = name;
    }
    String getData(){
        return name;
    }
}
