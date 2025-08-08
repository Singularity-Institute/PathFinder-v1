package com.flightplanner.models;

import java.util.Objects;

/**
 * Represents a geographic waypoint with navigation calculations.
 * Base class for all aviation navigation points.
 */
public class Waypoint {
    protected String name;
    protected double latitude;
    protected double longitude;
    protected Double altitude; // Optional altitude in feet
    protected String waypointType;
    
    // Earth radius in nautical miles
    private static final double EARTH_RADIUS_NM = 3440.065;
    
    /**
     * Create a waypoint with coordinates.
     * 
     * @param name Waypoint identifier
     * @param latitude Latitude in decimal degrees
     * @param longitude Longitude in decimal degrees
     * @param altitude Optional altitude in feet (can be null)
     * @param waypointType Type of waypoint (USER, NAMED, VOR, etc.)
     */
    public Waypoint(String name, double latitude, double longitude, Double altitude, String waypointType) {
        this.name = name;
        this.latitude = latitude;
        this.longitude = longitude;
        this.altitude = altitude;
        this.waypointType = waypointType != null ? waypointType : "USER";
    }
    
    /**
     * Create a waypoint without altitude.
     */
    public Waypoint(String name, double latitude, double longitude) {
        this(name, latitude, longitude, null, "USER");
    }
    
    /**
     * Calculate great circle distance to another waypoint in nautical miles.
     * 
     * @param other Target waypoint
     * @return Distance in nautical miles
     */
    public double distanceTo(Waypoint other) {
        double lat1 = Math.toRadians(this.latitude);
        double lon1 = Math.toRadians(this.longitude);
        double lat2 = Math.toRadians(other.latitude);
        double lon2 = Math.toRadians(other.longitude);
        
        double dlat = lat2 - lat1;
        double dlon = lon2 - lon1;
        
        double a = Math.sin(dlat / 2) * Math.sin(dlat / 2) +
                  Math.cos(lat1) * Math.cos(lat2) *
                  Math.sin(dlon / 2) * Math.sin(dlon / 2);
        
        double c = 2 * Math.asin(Math.sqrt(a));
        
        return EARTH_RADIUS_NM * c;
    }
    
    /**
     * Calculate bearing from this waypoint to another in degrees.
     * 
     * @param other Target waypoint
     * @return True bearing in degrees (0-360)
     */
    public double bearingTo(Waypoint other) {
        double lat1 = Math.toRadians(this.latitude);
        double lon1 = Math.toRadians(this.longitude);
        double lat2 = Math.toRadians(other.latitude);
        double lon2 = Math.toRadians(other.longitude);
        
        double dlon = lon2 - lon1;
        
        double y = Math.sin(dlon) * Math.cos(lat2);
        double x = Math.cos(lat1) * Math.sin(lat2) -
                  Math.sin(lat1) * Math.cos(lat2) * Math.cos(dlon);
        
        double bearing = Math.atan2(y, x);
        return (Math.toDegrees(bearing) + 360) % 360;
    }
    
    // Getters and setters
    public String getName() { return name; }
    public void setName(String name) { this.name = name; }
    
    public double getLatitude() { return latitude; }
    public void setLatitude(double latitude) { this.latitude = latitude; }
    
    public double getLongitude() { return longitude; }
    public void setLongitude(double longitude) { this.longitude = longitude; }
    
    public Double getAltitude() { return altitude; }
    public void setAltitude(Double altitude) { this.altitude = altitude; }
    
    public String getWaypointType() { return waypointType; }
    public void setWaypointType(String waypointType) { this.waypointType = waypointType; }
    
    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        Waypoint waypoint = (Waypoint) o;
        return Double.compare(waypoint.latitude, latitude) == 0 &&
               Double.compare(waypoint.longitude, longitude) == 0 &&
               Objects.equals(name, waypoint.name);
    }
    
    @Override
    public int hashCode() {
        return Objects.hash(name, 
                          Math.round(latitude * 1000000), 
                          Math.round(longitude * 1000000));
    }
    
    @Override
    public String toString() {
        return String.format("%s (%.4f°, %.4f°)", name, latitude, longitude);
    }
}