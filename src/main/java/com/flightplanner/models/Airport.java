package com.flightplanner.models;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;

/**
 * Represents an airport with runways and operational information.
 * Extends Waypoint to provide airport-specific functionality.
 */
public class Airport extends Waypoint {
    private String icaoCode;
    private String iataCode;
    private String airportType;
    private double elevationFt;
    private List<Runway> runways;
    
    /**
     * Create an airport with full parameters.
     * 
     * @param name Airport name
     * @param latitude Latitude in decimal degrees
     * @param longitude Longitude in decimal degrees
     * @param icaoCode ICAO airport code (4 letters)
     * @param iataCode IATA airport code (3 letters, optional)
     * @param airportType Airport type (PUBLIC, PRIVATE, MILITARY)
     * @param elevationFt Airport elevation in feet MSL
     * @param runways List of runways (optional)
     */
    public Airport(String name, double latitude, double longitude, String icaoCode,
                  String iataCode, String airportType, double elevationFt, List<Runway> runways) {
        super(name, latitude, longitude, elevationFt, "AIRPORT");
        this.icaoCode = icaoCode;
        this.iataCode = iataCode;
        this.airportType = airportType != null ? airportType : "PUBLIC";
        this.elevationFt = elevationFt;
        this.runways = runways != null ? new ArrayList<>(runways) : new ArrayList<>();
    }
    
    /**
     * Create an airport with basic parameters.
     */
    public Airport(String name, double latitude, double longitude, String icaoCode, String iataCode) {
        this(name, latitude, longitude, icaoCode, iataCode, "PUBLIC", 0.0, null);
    }
    
    /**
     * Add a runway to this airport.
     * 
     * @param runway Runway to add
     */
    public void addRunway(Runway runway) {
        if (runway != null) {
            this.runways.add(runway);
        }
    }
    
    /**
     * Get the longest runway at this airport.
     * 
     * @return Optional containing the longest runway, or empty if no runways
     */
    public Optional<Runway> getLongestRunway() {
        return runways.stream()
                     .max((r1, r2) -> Double.compare(r1.getLengthFt(), r2.getLengthFt()));
    }
    
    /**
     * Check if airport can accommodate aircraft with required runway length.
     * 
     * @param requiredRunwayLength Required runway length in feet
     * @return true if airport has a suitable runway
     */
    public boolean canAccommodateAircraft(double requiredRunwayLength) {
        Optional<Runway> longestRunway = getLongestRunway();
        return longestRunway.map(runway -> runway.getLengthFt() >= requiredRunwayLength)
                           .orElse(false);
    }
    
    /**
     * Get runways with at least the specified length.
     * 
     * @param minLength Minimum runway length in feet
     * @return List of suitable runways
     */
    public List<Runway> getRunwaysWithMinLength(double minLength) {
        return runways.stream()
                     .filter(runway -> runway.getLengthFt() >= minLength)
                     .collect(Collectors.toList());
    }
    
    // Getters and setters
    public String getIcaoCode() { return icaoCode; }
    public void setIcaoCode(String icaoCode) { this.icaoCode = icaoCode; }
    
    public String getIataCode() { return iataCode; }
    public void setIataCode(String iataCode) { this.iataCode = iataCode; }
    
    public String getAirportType() { return airportType; }
    public void setAirportType(String airportType) { this.airportType = airportType; }
    
    public double getElevationFt() { return elevationFt; }
    public void setElevationFt(double elevationFt) { this.elevationFt = elevationFt; }
    
    public List<Runway> getRunways() { return new ArrayList<>(runways); }
    public void setRunways(List<Runway> runways) { 
        this.runways = runways != null ? new ArrayList<>(runways) : new ArrayList<>(); 
    }
    
    @Override
    public String toString() {
        String codeStr = icaoCode;
        if (iataCode != null && !iataCode.isEmpty()) {
            codeStr += "/" + iataCode;
        }
        return String.format("%s (%s)", name, codeStr);
    }
}