package com.flightplanner.core;

import com.flightplanner.models.Waypoint;
import com.flightplanner.models.VORStation;
import com.flightplanner.models.Airport;
import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.HashMap;

/**
 * Represents a complete flight plan with waypoints and navigation data.
 */
public class FlightPlan {
    private final List<Waypoint> waypoints;
    private final double totalDistance;
    private final List<Segment> segments;
    
    /**
     * Create a flight plan with waypoints.
     * 
     * @param waypoints List of waypoints in order
     */
    public FlightPlan(List<Waypoint> waypoints) {
        this.waypoints = new ArrayList<>(waypoints);
        this.totalDistance = calculateTotalDistance();
        this.segments = createSegments();
    }
    
    /**
     * Calculate total distance of the flight plan.
     * 
     * @return Total distance in nautical miles
     */
    private double calculateTotalDistance() {
        if (waypoints.size() < 2) {
            return 0.0;
        }
        
        double total = 0.0;
        for (int i = 0; i < waypoints.size() - 1; i++) {
            total += waypoints.get(i).distanceTo(waypoints.get(i + 1));
        }
        return total;
    }
    
    /**
     * Create flight plan segments with navigation data.
     * 
     * @return List of segments
     */
    private List<Segment> createSegments() {
        List<Segment> segmentList = new ArrayList<>();
        
        for (int i = 0; i < waypoints.size() - 1; i++) {
            Waypoint from = waypoints.get(i);
            Waypoint to = waypoints.get(i + 1);
            
            Segment segment = new Segment(
                from,
                to,
                from.distanceTo(to),
                from.bearingTo(to),
                determineLegType(from, to)
            );
            segmentList.add(segment);
        }
        
        return segmentList;
    }
    
    /**
     * Determine the type of navigation leg.
     * 
     * @param from Source waypoint
     * @param to Destination waypoint
     * @return Leg type string
     */
    private String determineLegType(Waypoint from, Waypoint to) {
        if (from instanceof VORStation || to instanceof VORStation) {
            return "VOR";
        } else if (from instanceof Airport || to instanceof Airport) {
            return "TERMINAL";
        } else {
            return "DIRECT";
        }
    }
    
    /**
     * Calculate flight time given ground speed.
     * 
     * @param groundSpeedKnots Ground speed in knots
     * @return Flight time in hours
     */
    public double getFlightTime(double groundSpeedKnots) {
        return groundSpeedKnots > 0 ? totalDistance / groundSpeedKnots : 0.0;
    }
    
    /**
     * Export flight plan to map format for JSON serialization.
     * 
     * @return Map representation of flight plan
     */
    public Map<String, Object> exportToMap() {
        Map<String, Object> data = new HashMap<>();
        
        // Waypoints
        List<Map<String, Object>> waypointList = new ArrayList<>();
        for (Waypoint wp : waypoints) {
            Map<String, Object> wpData = new HashMap<>();
            wpData.put("name", wp.getName());
            wpData.put("lat", wp.getLatitude());
            wpData.put("lon", wp.getLongitude());
            wpData.put("alt", wp.getAltitude());
            wpData.put("type", wp.getWaypointType());
            waypointList.add(wpData);
        }
        data.put("waypoints", waypointList);
        
        // Total distance
        data.put("total_distance", totalDistance);
        
        // Segments
        List<Map<String, Object>> segmentList = new ArrayList<>();
        for (Segment segment : segments) {
            Map<String, Object> segData = new HashMap<>();
            segData.put("from", segment.getFrom().getName());
            segData.put("to", segment.getTo().getName());
            segData.put("distance", segment.getDistance());
            segData.put("bearing", segment.getBearing());
            segData.put("type", segment.getLegType());
            segmentList.add(segData);
        }
        data.put("segments", segmentList);
        
        return data;
    }
    
    /**
     * Export flight plan to JSON string.
     * 
     * @return JSON representation of flight plan
     */
    public String exportToJson() {
        Gson gson = new GsonBuilder().setPrettyPrinting().create();
        return gson.toJson(exportToMap());
    }
    
    /**
     * Export flight plan to GPX format.
     * 
     * @return GPX XML string
     */
    public String exportToGPX() {
        StringBuilder gpx = new StringBuilder();
        gpx.append("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
        gpx.append("<gpx version=\"1.1\" creator=\"FlightPlannerJava\">\n");
        gpx.append("  <rte>\n");
        gpx.append("    <name>Flight Plan</name>\n");
        
        for (Waypoint wp : waypoints) {
            gpx.append(String.format("    <rtept lat=\"%.6f\" lon=\"%.6f\">\n", 
                                   wp.getLatitude(), wp.getLongitude()));
            gpx.append(String.format("      <name>%s</name>\n", wp.getName()));
            if (wp.getAltitude() != null) {
                // Convert feet to meters
                double elevationM = wp.getAltitude() * 0.3048;
                gpx.append(String.format("      <ele>%.1f</ele>\n", elevationM));
            }
            gpx.append("    </rtept>\n");
        }
        
        gpx.append("  </rte>\n");
        gpx.append("</gpx>\n");
        
        return gpx.toString();
    }
    
    // Getters
    public List<Waypoint> getWaypoints() { return new ArrayList<>(waypoints); }
    public double getTotalDistance() { return totalDistance; }
    public List<Segment> getSegments() { return new ArrayList<>(segments); }
    
    /**
     * Represents a segment between two waypoints.
     */
    public static class Segment {
        private final Waypoint from;
        private final Waypoint to;
        private final double distance;
        private final double bearing;
        private final String legType;
        
        public Segment(Waypoint from, Waypoint to, double distance, double bearing, String legType) {
            this.from = from;
            this.to = to;
            this.distance = distance;
            this.bearing = bearing;
            this.legType = legType;
        }
        
        // Getters
        public Waypoint getFrom() { return from; }
        public Waypoint getTo() { return to; }
        public double getDistance() { return distance; }
        public double getBearing() { return bearing; }
        public String getLegType() { return legType; }
        
        @Override
        public String toString() {
            return String.format("%s → %s: %.1f nm @ %.0f°", 
                               from.getName(), to.getName(), distance, bearing);
        }
    }
}