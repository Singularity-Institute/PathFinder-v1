package com.flightplanner.utils;

import com.flightplanner.models.Waypoint;
import com.flightplanner.models.VORStation;
import com.flightplanner.models.Airport;
import java.util.List;
import java.util.ArrayList;
import java.util.regex.Pattern;
import java.util.regex.Matcher;

/**
 * Utility class for coordinate conversion and Mediterranean waypoint data generation.
 * Converts DMS (Degrees Minutes Seconds) format to decimal degrees.
 */
public class CoordinateConverter {
    
    // DMS regex pattern for parsing coordinates like "36°45'30"N"
    private static final Pattern DMS_PATTERN = Pattern.compile(
        "^(\\d+)°(\\d+)'(\\d+(?:\\.\\d+)?)\"([NS]).*?(\\d+)°(\\d+)'(\\d+(?:\\.\\d+)?)\"([EW])$"
    );
    
    /**
     * Parse a DMS coordinate string and convert to decimal degrees.
     * Supports formats like:
     * - "36°45'30"N 10°15'45"E"
     * - "40°30'00"N 025°15'30"E"
     * 
     * @param dmsString DMS coordinate string
     * @return Array of [latitude, longitude] in decimal degrees
     * @throws IllegalArgumentException if format is invalid
     */
    public static double[] parseDMSCoordinate(String dmsString) {
        Matcher matcher = DMS_PATTERN.matcher(dmsString.trim());
        
        if (!matcher.matches()) {
            throw new IllegalArgumentException("Invalid DMS format: " + dmsString);
        }
        
        // Parse latitude
        int latDeg = Integer.parseInt(matcher.group(1));
        int latMin = Integer.parseInt(matcher.group(2));
        double latSec = Double.parseDouble(matcher.group(3));
        String latDir = matcher.group(4);
        
        // Parse longitude
        int lonDeg = Integer.parseInt(matcher.group(5));
        int lonMin = Integer.parseInt(matcher.group(6));
        double lonSec = Double.parseDouble(matcher.group(7));
        String lonDir = matcher.group(8);
        
        // Convert to decimal degrees
        double latitude = latDeg + latMin / 60.0 + latSec / 3600.0;
        double longitude = lonDeg + lonMin / 60.0 + lonSec / 3600.0;
        
        // Apply direction
        if ("S".equals(latDir)) latitude = -latitude;
        if ("W".equals(lonDir)) longitude = -longitude;
        
        return new double[]{latitude, longitude};
    }
    
    /**
     * Create Mediterranean airports data.
     * 
     * @return List of Mediterranean airports
     */
    public static List<Airport> createMediterraneanAirports() {
        List<Airport> airports = new ArrayList<>();
        
        // Tunis Carthage Airport (DTTA)
        airports.add(new Airport(
            "Tunis Carthage International Airport",
            36.851, 10.227, "DTTA", "TUN"
        ));
        
        // Santorini Airport (LGSR)
        airports.add(new Airport(
            "Santorini Airport",
            36.399, 25.479, "LGSR", "JTR"
        ));
        
        return airports;
    }
    
    /**
     * Create Mediterranean VOR stations data.
     * 
     * @return List of Mediterranean VOR stations
     */
    public static List<VORStation> createMediterraneanVORs() {
        List<VORStation> vors = new ArrayList<>();
        
        // TUC VOR
        vors.add(new VORStation(
            "TUC", 36.8, 10.2, 115.5, "TUC", null, 0.0, 300.0, false
        ));
        
        // MIL VOR  
        vors.add(new VORStation(
            "MIL", 37.5, 15.0, 116.2, "MIL", null, 0.0, 300.0, false
        ));
        
        return vors;
    }
    
    /**
     * Create Mediterranean waypoints from DMS coordinates.
     * These are the 14 waypoints provided by the user.
     * 
     * @return List of Mediterranean waypoints
     */
    public static List<Waypoint> createMediterraneanWaypoints() {
        List<Waypoint> waypoints = new ArrayList<>();
        
        // Mediterranean waypoints with their DMS coordinates
        String[][] waypointData = {
            {"WP1", "36°45'30\"N 10°15'45\"E"},
            {"WP2", "37°12'15\"N 11°30'20\"E"},
            {"WP3", "37°45'10\"N 12°45'35\"E"},
            {"WP4", "38°15'25\"N 14°10'50\"E"},
            {"WP5", "38°30'40\"N 15°25'15\"E"},
            {"WP6", "38°45'55\"N 16°40'30\"E"},
            {"WP7", "39°10'20\"N 18°15'45\"E"},
            {"WP8", "39°25'35\"N 19°30'10\"E"},
            {"WP9", "39°40'50\"N 20°45'25\"E"},
            {"WP10", "40°15'15\"N 22°10'40\"E"},
            {"WP11", "40°30'30\"N 23°25'55\"E"},
            {"WP12", "40°45'45\"N 24°40'20\"E"},
            {"WP13", "36°30'00\"N 025°15'30\"E"},
            {"WP14", "36°24'00\"N 025°28'30\"E"}
        };
        
        for (String[] data : waypointData) {
            String name = data[0];
            String dmsCoord = data[1];
            
            try {
                double[] coords = parseDMSCoordinate(dmsCoord);
                waypoints.add(new Waypoint(name, coords[0], coords[1], 35000.0, "WAYPOINT"));
            } catch (IllegalArgumentException e) {
                System.err.println("Failed to parse coordinates for " + name + ": " + e.getMessage());
            }
        }
        
        return waypoints;
    }
    
    /**
     * Convert decimal degrees to DMS format string.
     * 
     * @param latitude Latitude in decimal degrees
     * @param longitude Longitude in decimal degrees
     * @return DMS formatted string
     */
    public static String toDMSString(double latitude, double longitude) {
        return formatDMS(latitude, true) + " " + formatDMS(longitude, false);
    }
    
    /**
     * Format a single coordinate to DMS.
     * 
     * @param coord Coordinate in decimal degrees
     * @param isLatitude true for latitude, false for longitude
     * @return DMS formatted string
     */
    private static String formatDMS(double coord, boolean isLatitude) {
        boolean isNegative = coord < 0;
        coord = Math.abs(coord);
        
        int degrees = (int) coord;
        double minutesFloat = (coord - degrees) * 60;
        int minutes = (int) minutesFloat;
        double seconds = (minutesFloat - minutes) * 60;
        
        char direction;
        if (isLatitude) {
            direction = isNegative ? 'S' : 'N';
        } else {
            direction = isNegative ? 'W' : 'E';
        }
        
        return String.format("%02d°%02d'%06.3f\"%c", degrees, minutes, seconds, direction);
    }
    
    /**
     * Generate sample Mediterranean data for testing.
     * 
     * @return MediterraneanData object containing all waypoints, airports, and VORs
     */
    public static MediterraneanData generateMediterraneanData() {
        return new MediterraneanData(
            createMediterraneanWaypoints(),
            createMediterraneanAirports(),
            createMediterraneanVORs()
        );
    }
    
    /**
     * Container class for Mediterranean navigation data.
     */
    public static class MediterraneanData {
        private final List<Waypoint> waypoints;
        private final List<Airport> airports;
        private final List<VORStation> vors;
        
        public MediterraneanData(List<Waypoint> waypoints, List<Airport> airports, List<VORStation> vors) {
            this.waypoints = waypoints;
            this.airports = airports;
            this.vors = vors;
        }
        
        public List<Waypoint> getWaypoints() { return waypoints; }
        public List<Airport> getAirports() { return airports; }
        public List<VORStation> getVors() { return vors; }
    }
}