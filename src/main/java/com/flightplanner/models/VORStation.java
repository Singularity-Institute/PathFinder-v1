package com.flightplanner.models;

/**
 * Represents a VOR (VHF Omnidirectional Range) navigation station.
 * Extends Waypoint with VOR-specific properties and navigation methods.
 */
public class VORStation extends Waypoint {
    private double frequency;
    private String identifier;
    private double magneticVariation;
    private double rangeNm;
    private boolean dmeAvailable;
    
    /**
     * Create a VOR station with full parameters.
     * 
     * @param name Station name
     * @param latitude Latitude in decimal degrees
     * @param longitude Longitude in decimal degrees
     * @param frequency VOR frequency in MHz
     * @param identifier VOR identifier (3-letter code)
     * @param altitude Optional altitude in feet
     * @param magneticVariation Magnetic variation in degrees
     * @param rangeNm Operational range in nautical miles
     * @param dmeAvailable Whether DME (Distance Measuring Equipment) is available
     */
    public VORStation(String name, double latitude, double longitude, 
                     double frequency, String identifier, Double altitude,
                     double magneticVariation, double rangeNm, boolean dmeAvailable) {
        super(name, latitude, longitude, altitude, "VOR");
        this.frequency = frequency;
        this.identifier = identifier;
        this.magneticVariation = magneticVariation;
        this.rangeNm = rangeNm;
        this.dmeAvailable = dmeAvailable;
    }
    
    /**
     * Create a VOR station with default parameters.
     */
    public VORStation(String name, double latitude, double longitude, 
                     double frequency, String identifier) {
        this(name, latitude, longitude, frequency, identifier, null, 0.0, 100.0, false);
    }
    
    /**
     * Check if another waypoint is within VOR reception range.
     * 
     * @param other Waypoint to check
     * @return true if within range
     */
    public boolean isInRange(Waypoint other) {
        double distance = this.distanceTo(other);
        return distance <= rangeNm;
    }
    
    /**
     * Get the radial FROM the VOR to the other waypoint.
     * Radial is the magnetic bearing FROM the VOR station.
     * 
     * @param other Target waypoint
     * @return Radial in degrees (0-360)
     */
    public double getRadial(Waypoint other) {
        double bearing = this.bearingTo(other);
        // Adjust for magnetic variation
        double radial = (bearing + magneticVariation) % 360;
        if (radial < 0) radial += 360;
        return radial;
    }
    
    /**
     * Get bearing TO the VOR station from another waypoint.
     * 
     * @param other Source waypoint
     * @return Magnetic bearing TO the station in degrees
     */
    public double getBearingToStation(Waypoint other) {
        double bearing = other.bearingTo(this);
        // Adjust for magnetic variation
        double magneticBearing = (bearing + magneticVariation) % 360;
        if (magneticBearing < 0) magneticBearing += 360;
        return magneticBearing;
    }
    
    // Getters and setters
    public double getFrequency() { return frequency; }
    public void setFrequency(double frequency) { this.frequency = frequency; }
    
    public String getIdentifier() { return identifier; }
    public void setIdentifier(String identifier) { this.identifier = identifier; }
    
    public double getMagneticVariation() { return magneticVariation; }
    public void setMagneticVariation(double magneticVariation) { this.magneticVariation = magneticVariation; }
    
    public double getRangeNm() { return rangeNm; }
    public void setRangeNm(double rangeNm) { this.rangeNm = rangeNm; }
    
    public boolean isDmeAvailable() { return dmeAvailable; }
    public void setDmeAvailable(boolean dmeAvailable) { this.dmeAvailable = dmeAvailable; }
    
    @Override
    public String toString() {
        return String.format("%s VOR (%s) - %.2f MHz", identifier, name, frequency);
    }
}