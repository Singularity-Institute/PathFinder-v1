package com.flightplanner.core;

import com.flightplanner.models.Waypoint;
import com.flightplanner.models.VORStation;
import com.flightplanner.models.Airport;
import com.flightplanner.algorithms.PathOptimizer;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;
import java.io.FileWriter;
import java.io.IOException;

/**
 * Main flight planner class for creating and optimizing flight plans.
 * Manages airports, VOR stations, waypoints, and provides route planning functionality.
 */
public class FlightPlanner {
    private final List<Airport> airports;
    private final List<VORStation> vorStations;
    private final List<Waypoint> waypoints;
    private PathOptimizer optimizer;
    
    /**
     * Create a new flight planner.
     */
    public FlightPlanner() {
        this.airports = new ArrayList<>();
        this.vorStations = new ArrayList<>();
        this.waypoints = new ArrayList<>();
        this.optimizer = null;
    }
    
    /**
     * Add an airport to the planner database.
     * 
     * @param airport Airport to add
     */
    public void addAirport(Airport airport) {
        if (airport != null) {
            airports.add(airport);
            updateOptimizer();
        }
    }
    
    /**
     * Add a VOR station to the planner database.
     * 
     * @param vor VOR station to add
     */
    public void addVORStation(VORStation vor) {
        if (vor != null) {
            vorStations.add(vor);
            updateOptimizer();
        }
    }
    
    /**
     * Add a waypoint to the planner database.
     * 
     * @param waypoint Waypoint to add
     */
    public void addWaypoint(Waypoint waypoint) {
        if (waypoint != null) {
            waypoints.add(waypoint);
            updateOptimizer();
        }
    }
    
    /**
     * Update the path optimizer with current data.
     */
    private void updateOptimizer() {
        List<Waypoint> allWaypoints = new ArrayList<>(waypoints);
        allWaypoints.addAll(airports);
        this.optimizer = new PathOptimizer(allWaypoints, vorStations);
    }
    
    /**
     * Plan a direct route between two airports.
     * 
     * @param departure Departure airport
     * @param arrival Arrival airport
     * @return Flight plan with direct route
     */
    public FlightPlan planDirectRoute(Airport departure, Airport arrival) {
        List<Waypoint> route = new ArrayList<>();
        route.add(departure);
        route.add(arrival);
        return new FlightPlan(route);
    }
    
    /**
     * Plan an optimized route with waypoint and VOR optimization.
     * 
     * @param departure Departure airport
     * @param arrival Arrival airport
     * @param intermediateWaypoints Optional intermediate waypoints
     * @param useVorPreferred Whether to prefer VOR stations in routing
     * @return Optimized flight plan
     */
    public FlightPlan planOptimizedRoute(Airport departure, Airport arrival, 
                                       List<Waypoint> intermediateWaypoints, 
                                       boolean useVorPreferred) {
        if (optimizer == null) {
            updateOptimizer();
        }
        
        List<Waypoint> optimalPath;
        
        if (intermediateWaypoints == null || intermediateWaypoints.isEmpty()) {
            // Find optimal path using A*
            optimalPath = optimizer.findOptimalPath(departure, arrival, useVorPreferred);
        } else {
            // Optimize the sequence of intermediate waypoints
            List<Waypoint> allPoints = new ArrayList<>();
            allPoints.add(departure);
            allPoints.addAll(intermediateWaypoints);
            allPoints.add(arrival);
            
            List<Waypoint> optimalSequence = optimizer.optimizeWaypointSequence(allPoints);
            
            // Create optimized path through all waypoints
            optimalPath = new ArrayList<>();
            for (int i = 0; i < optimalSequence.size() - 1; i++) {
                List<Waypoint> segmentPath = optimizer.findOptimalPath(
                    optimalSequence.get(i), 
                    optimalSequence.get(i + 1), 
                    useVorPreferred
                );
                
                if (i == 0) {
                    optimalPath.addAll(segmentPath);
                } else {
                    // Skip duplicate waypoint
                    optimalPath.addAll(segmentPath.subList(1, segmentPath.size()));
                }
            }
        }
        
        return new FlightPlan(optimalPath);
    }
    
    /**
     * Plan an optimized route with default VOR preference.
     * 
     * @param departure Departure airport
     * @param arrival Arrival airport
     * @return Optimized flight plan
     */
    public FlightPlan planOptimizedRoute(Airport departure, Airport arrival) {
        return planOptimizedRoute(departure, arrival, null, true);
    }
    
    /**
     * Plan a route optimized for VOR navigation.
     * 
     * @param departure Departure airport
     * @param arrival Arrival airport
     * @return VOR-optimized flight plan and covering VOR stations
     */
    public VOROptimizedResult planVORRoute(Airport departure, Airport arrival) {
        if (optimizer == null) {
            updateOptimizer();
        }
        
        PathOptimizer.PathWithVORCoverage result = optimizer.findVORRoute(departure, arrival);
        FlightPlan plan = new FlightPlan(result.getPath());
        
        return new VOROptimizedResult(plan, result.getCoverageVORs());
    }
    
    /**
     * Find alternate airports within specified distance.
     * 
     * @param primaryAirport Primary airport
     * @param maxDistanceNm Maximum distance in nautical miles
     * @return List of alternate airports with distances
     */
    public List<AlternateAirport> findAlternateAirports(Airport primaryAirport, double maxDistanceNm) {
        return airports.stream()
                      .filter(airport -> !airport.equals(primaryAirport))
                      .map(airport -> new AlternateAirport(airport, primaryAirport.distanceTo(airport)))
                      .filter(alt -> alt.getDistance() <= maxDistanceNm)
                      .sorted((a1, a2) -> Double.compare(a1.getDistance(), a2.getDistance()))
                      .collect(Collectors.toList());
    }
    
    /**
     * Get VOR stations that provide coverage for a waypoint.
     * 
     * @param waypoint Waypoint to check
     * @return List of VOR stations providing coverage
     */
    public List<VORStation> getVORCoverage(Waypoint waypoint) {
        return vorStations.stream()
                         .filter(vor -> vor.isInRange(waypoint))
                         .collect(Collectors.toList());
    }
    
    /**
     * Export flight plan to file.
     * 
     * @param flightPlan Flight plan to export
     * @param filename Output filename
     * @param format Export format ("json" or "gpx")
     * @throws IOException If file writing fails
     */
    public void exportFlightPlan(FlightPlan flightPlan, String filename, String format) throws IOException {
        String content;
        
        switch (format.toLowerCase()) {
            case "json":
                content = flightPlan.exportToJson();
                break;
            case "gpx":
                content = flightPlan.exportToGPX();
                break;
            default:
                throw new IllegalArgumentException("Unsupported export format: " + format);
        }
        
        try (FileWriter writer = new FileWriter(filename)) {
            writer.write(content);
        }
    }
    
    // Getters
    public List<Airport> getAirports() { return new ArrayList<>(airports); }
    public List<VORStation> getVORStations() { return new ArrayList<>(vorStations); }
    public List<Waypoint> getWaypoints() { return new ArrayList<>(waypoints); }
    public PathOptimizer getOptimizer() { return optimizer; }
    
    /**
     * Result class for VOR-optimized routes.
     */
    public static class VOROptimizedResult {
        private final FlightPlan flightPlan;
        private final List<VORStation> coverageVORs;
        
        public VOROptimizedResult(FlightPlan flightPlan, List<VORStation> coverageVORs) {
            this.flightPlan = flightPlan;
            this.coverageVORs = new ArrayList<>(coverageVORs);
        }
        
        public FlightPlan getFlightPlan() { return flightPlan; }
        public List<VORStation> getCoverageVORs() { return new ArrayList<>(coverageVORs); }
    }
    
    /**
     * Represents an alternate airport with distance.
     */
    public static class AlternateAirport {
        private final Airport airport;
        private final double distance;
        
        public AlternateAirport(Airport airport, double distance) {
            this.airport = airport;
            this.distance = distance;
        }
        
        public Airport getAirport() { return airport; }
        public double getDistance() { return distance; }
        
        @Override
        public String toString() {
            return String.format("%s - %.1f nm", airport.toString(), distance);
        }
    }
}