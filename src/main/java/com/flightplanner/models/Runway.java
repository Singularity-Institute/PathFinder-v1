package com.flightplanner.models;

/**
 * Represents an airport runway with its characteristics.
 */
public class Runway {
    private String identifier;
    private double lengthFt;
    private double widthFt;
    private double heading;
    private String surfaceType;
    
    /**
     * Create a runway with full parameters.
     * 
     * @param identifier Runway identifier (e.g., "09/27", "16L/34R")
     * @param lengthFt Runway length in feet
     * @param widthFt Runway width in feet
     * @param heading Runway heading in degrees
     * @param surfaceType Surface type (ASPH, CONC, GRASS, etc.)
     */
    public Runway(String identifier, double lengthFt, double widthFt, 
                  double heading, String surfaceType) {
        this.identifier = identifier;
        this.lengthFt = lengthFt;
        this.widthFt = widthFt;
        this.heading = heading;
        this.surfaceType = surfaceType != null ? surfaceType : "ASPH";
    }
    
    /**
     * Create a runway with default surface type.
     */
    public Runway(String identifier, double lengthFt, double widthFt, double heading) {
        this(identifier, lengthFt, widthFt, heading, "ASPH");
    }
    
    // Getters and setters
    public String getIdentifier() { return identifier; }
    public void setIdentifier(String identifier) { this.identifier = identifier; }
    
    public double getLengthFt() { return lengthFt; }
    public void setLengthFt(double lengthFt) { this.lengthFt = lengthFt; }
    
    public double getWidthFt() { return widthFt; }
    public void setWidthFt(double widthFt) { this.widthFt = widthFt; }
    
    public double getHeading() { return heading; }
    public void setHeading(double heading) { this.heading = heading; }
    
    public String getSurfaceType() { return surfaceType; }
    public void setSurfaceType(String surfaceType) { this.surfaceType = surfaceType; }
    
    @Override
    public String toString() {
        return String.format("Runway %s (%,.0f ft, %s)", identifier, lengthFt, surfaceType);
    }
}