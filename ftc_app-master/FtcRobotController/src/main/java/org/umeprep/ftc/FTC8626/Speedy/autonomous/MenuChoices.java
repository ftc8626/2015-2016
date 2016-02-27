package org.umeprep.ftc.FTC8626.Speedy.autonomous;

/**
 * Created by Andre on 1/25/2016.
 */
public class MenuChoices {
    private String alliance = "default";
    private String startingPosition = "default";
    private int startDelay = 0;

    public String getAlliance() { return this.alliance; }
    public void setAlliance(String alliance) { this.alliance = alliance; }

    public String getStartingPosition() { return this.startingPosition; }
    public void setStartingPosition(String startingPosition) { this.startingPosition = startingPosition; }
   
    public int getStartDelay() { return this.startDelay; }
    public void setStartDelay(String startDelay) {
        try {
            this.startDelay = Integer.parseInt(startDelay);
        } catch (NumberFormatException e) {
            //Will Throw exception!
            //do something! anything to handle the exception.
            this.startDelay = 0;
        }

    }
}
