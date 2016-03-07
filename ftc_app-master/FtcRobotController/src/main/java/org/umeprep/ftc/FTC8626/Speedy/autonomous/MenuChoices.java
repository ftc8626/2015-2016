package org.umeprep.ftc.FTC8626.Speedy.autonomous;

/**
 * Created by Andre on 1/25/2016.
 */
public class MenuChoices {
    private String alliance = "default";
    private String startingPosition = "default";
    private String shouldPark = "default";
    private int startDelay = 0;

    public String getAlliance() { return this.alliance; }
    public void setAlliance(String allianceParameter) { this.alliance = allianceParameter; }

    public String getStartingPosition() { return this.startingPosition; }
    public void setStartingPosition(String startingPositionParameter) { this.startingPosition = startingPositionParameter; }

    public String getShouldPark() { return this.shouldPark; }
    public void setShouldPark(String shouldParkParameter) { this.shouldPark = shouldParkParameter; }
   
    public int getStartDelay() { return this.startDelay; }
    public void setStartDelay(String startDelayParameter) {
        try {
            this.startDelay = Integer.parseInt(startDelayParameter);
        } catch (NumberFormatException e) {
            //Will Throw exception!
            //do something! anything to handle the exception.
            this.startDelay = 0;
        }
    }
}
