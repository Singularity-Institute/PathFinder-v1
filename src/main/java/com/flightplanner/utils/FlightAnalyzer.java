package com.flightplanner.utils;

import com.flightplanner.core.FlightPlan;
import com.flightplanner.models.Waypoint;
import com.flightplanner.models.VORStation;
import com.flightplanner.models.Airport;
import java.util.List;
import java.util.Map;
import java.util.HashMap;
import java.util.stream.Collectors;

/**
 * Utility class for analyzing flight plans and providing detailed metrics.
 * Provides comprehensive analysis capabilities similar to the Python version.
 */
public class FlightAnalyzer {
    
    /**
     * Generate detailed analysis report for a flight plan.
     * 
     * @param plan Flight plan to analyze
     * @param planName Name/description of the plan
     * @return Analysis report as formatted string
     */
    public static String generateAnalysisReport(FlightPlan plan, String planName) {
        StringBuilder report = new StringBuilder();
        
        report.append("=".repeat(50)).append("\n");
        report.append("FLIGHT PLAN ANALYSIS: ").append(planName).append("\n");
        report.append("=".repeat(50)).append("\n\n");
        
        // Basic statistics
        appendBasicStatistics(report, plan);
        
        // Route details
        appendRouteDetails(report, plan);
        
        // Segment analysis
        appendSegmentAnalysis(report, plan);
        
        // Navigation analysis
        appendNavigationAnalysis(report, plan);
        
        return report.toString();
    }
    
    /**
     * Append basic flight plan statistics to report.
     */
    private static void appendBasicStatistics(StringBuilder report, FlightPlan plan) {
        report.append("BASIC STATISTICS:\n");
        report.append("-".repeat(20)).append("\n");
        report.append(String.format("Total Distance:      %.1f nm\n", plan.getTotalDistance()));
        report.append(String.format("Number of Waypoints: %d\n", plan.getWaypoints().size()));
        report.append(String.format("Number of Segments:  %d\n", plan.getSegments().size()));
        
        // Flight time estimates for different speeds
        report.append("\nFlight Time Estimates:\n");
        int[] speeds = {350, 400, 450, 500, 550};
        for (int speed : speeds) {
            double time = plan.getFlightTime(speed);
            int hours = (int) time;
            int minutes = (int) ((time - hours) * 60);
            report.append(String.format("  @ %d kts: %d:%02d\n", speed, hours, minutes));
        }
        report.append("\n");
    }
    
    /**
     * Append detailed route information to report.
     */
    private static void appendRouteDetails(StringBuilder report, FlightPlan plan) {
        report.append("ROUTE DETAILS:\n");
        report.append("-".repeat(15)).append("\n");
        
        List<Waypoint> waypoints = plan.getWaypoints();
        for (int i = 0; i < waypoints.size(); i++) {
            Waypoint wp = waypoints.get(i);
            String prefix = (i == 0) ? "START: " : 
                          (i == waypoints.size() - 1) ? "END:   " : 
                          String.format("WP%02d:  ", i);
            
            report.append(String.format("%s%-30s (%.6f°, %.6f°) [%s]\n", 
                prefix, wp.getName(), wp.getLatitude(), wp.getLongitude(), wp.getWaypointType()));
        }
        report.append("\n");
    }
    
    /**
     * Append segment-by-segment analysis to report.
     */
    private static void appendSegmentAnalysis(StringBuilder report, FlightPlan plan) {
        report.append("SEGMENT ANALYSIS:\n");
        report.append("-".repeat(18)).append("\n");
        
        List<FlightPlan.Segment> segments = plan.getSegments();
        double totalDistance = 0;
        
        for (int i = 0; i < segments.size(); i++) {
            FlightPlan.Segment segment = segments.get(i);
            totalDistance += segment.getDistance();
            
            report.append(String.format("Leg %02d: %s → %s\n", 
                i + 1, segment.getFrom().getName(), segment.getTo().getName()));
            report.append(String.format("        Distance: %.1f nm\n", segment.getDistance()));
            report.append(String.format("        Bearing:  %.0f°\n", segment.getBearing()));
            report.append(String.format("        Type:     %s\n", segment.getLegType()));
            report.append(String.format("        Progress: %.1f nm (%.1f%%)\n", 
                totalDistance, (totalDistance / plan.getTotalDistance()) * 100));
            report.append("\n");
        }
    }
    
    /**
     * Append navigation-specific analysis to report.
     */
    private static void appendNavigationAnalysis(StringBuilder report, FlightPlan plan) {
        report.append("NAVIGATION ANALYSIS:\n");
        report.append("-".repeat(20)).append("\n");
        
        // Analyze waypoint types
        Map<String, Integer> typeCount = new HashMap<>();
        Map<String, Double> typeDistance = new HashMap<>();
        
        for (FlightPlan.Segment segment : plan.getSegments()) {
            String legType = segment.getLegType();
            typeCount.put(legType, typeCount.getOrDefault(legType, 0) + 1);
            typeDistance.put(legType, typeDistance.getOrDefault(legType, 0.0) + segment.getDistance());
        }
        
        report.append("Leg Type Distribution:\n");
        for (Map.Entry<String, Integer> entry : typeCount.entrySet()) {
            String type = entry.getKey();
            int count = entry.getValue();
            double distance = typeDistance.get(type);
            double percentage = (distance / plan.getTotalDistance()) * 100;
            
            report.append(String.format("  %-10s: %d legs, %.1f nm (%.1f%%)\n", 
                type, count, distance, percentage));
        }
        
        // Analyze segment lengths
        report.append("\nSegment Length Analysis:\n");
        List<Double> distances = plan.getSegments().stream()
            .map(FlightPlan.Segment::getDistance)
            .sorted()
            .collect(Collectors.toList());
        
        if (!distances.isEmpty()) {
            report.append(String.format("  Shortest segment: %.1f nm\n", distances.get(0)));
            report.append(String.format("  Longest segment:  %.1f nm\n", distances.get(distances.size() - 1)));
            report.append(String.format("  Average segment:  %.1f nm\n", 
                distances.stream().mapToDouble(Double::doubleValue).average().orElse(0.0)));
            
            // Median
            double median = distances.size() % 2 == 0 ?
                (distances.get(distances.size() / 2 - 1) + distances.get(distances.size() / 2)) / 2 :
                distances.get(distances.size() / 2);
            report.append(String.format("  Median segment:   %.1f nm\n", median));
        }
        
        report.append("\n");
    }
    
    /**
     * Compare two flight plans and generate comparison report.
     * 
     * @param plan1 First flight plan
     * @param plan1Name Name of first plan
     * @param plan2 Second flight plan  
     * @param plan2Name Name of second plan
     * @return Comparison report as formatted string
     */
    public static String compareFlightPlans(FlightPlan plan1, String plan1Name, 
                                          FlightPlan plan2, String plan2Name) {
        StringBuilder report = new StringBuilder();
        
        report.append("=".repeat(60)).append("\n");
        report.append("FLIGHT PLAN COMPARISON\n");
        report.append("=".repeat(60)).append("\n\n");
        
        report.append(String.format("Plan A: %s\n", plan1Name));
        report.append(String.format("Plan B: %s\n\n", plan2Name));
        
        // Distance comparison
        double dist1 = plan1.getTotalDistance();
        double dist2 = plan2.getTotalDistance();
        double distDiff = dist2 - dist1;
        double distPercent = (distDiff / dist1) * 100;
        
        report.append("DISTANCE COMPARISON:\n");
        report.append("-".repeat(20)).append("\n");
        report.append(String.format("Plan A Distance: %.1f nm\n", dist1));
        report.append(String.format("Plan B Distance: %.1f nm\n", dist2));
        report.append(String.format("Difference:      %+.1f nm (%.1f%%)\n\n", distDiff, distPercent));
        
        // Waypoint comparison
        int wp1 = plan1.getWaypoints().size();
        int wp2 = plan2.getWaypoints().size();
        
        report.append("WAYPOINT COMPARISON:\n");
        report.append("-".repeat(20)).append("\n");
        report.append(String.format("Plan A Waypoints: %d\n", wp1));
        report.append(String.format("Plan B Waypoints: %d\n", wp2));
        report.append(String.format("Difference:       %+d waypoints\n\n", wp2 - wp1));
        
        // Time comparison at different speeds
        report.append("FLIGHT TIME COMPARISON:\n");
        report.append("-".repeat(23)).append("\n");
        report.append(String.format("%-10s %-10s %-10s %-10s\n", "Speed", "Plan A", "Plan B", "Diff"));
        report.append("-".repeat(40)).append("\n");
        
        int[] speeds = {400, 450, 500};
        for (int speed : speeds) {
            double time1 = plan1.getFlightTime(speed);
            double time2 = plan2.getFlightTime(speed);
            double timeDiff = time2 - time1;
            
            report.append(String.format("%-10d %-10s %-10s %+.1f min\n", 
                speed, formatTime(time1), formatTime(time2), timeDiff * 60));
        }
        
        return report.toString();
    }
    
    /**
     * Format time in hours to HH:MM format.
     */
    private static String formatTime(double hours) {
        int h = (int) hours;
        int m = (int) ((hours - h) * 60);
        return String.format("%d:%02d", h, m);
    }
    
    /**
     * Calculate fuel consumption estimate for a flight plan.
     * 
     * @param plan Flight plan
     * @param groundSpeed Ground speed in knots
     * @param fuelFlowGPH Fuel flow in gallons per hour
     * @return Estimated fuel consumption in gallons
     */
    public static double estimateFuelConsumption(FlightPlan plan, double groundSpeed, double fuelFlowGPH) {
        double flightTime = plan.getFlightTime(groundSpeed);
        return flightTime * fuelFlowGPH;
    }
    
    /**
     * Generate performance metrics for a flight plan.
     * 
     * @param plan Flight plan to analyze
     * @return Map of performance metrics
     */
    public static Map<String, Double> calculatePerformanceMetrics(FlightPlan plan) {
        Map<String, Double> metrics = new HashMap<>();
        
        List<FlightPlan.Segment> segments = plan.getSegments();
        List<Double> distances = segments.stream()
            .map(FlightPlan.Segment::getDistance)
            .collect(Collectors.toList());
        
        metrics.put("totalDistance", plan.getTotalDistance());
        metrics.put("segmentCount", (double) segments.size());
        metrics.put("waypointCount", (double) plan.getWaypoints().size());
        
        if (!distances.isEmpty()) {
            metrics.put("averageSegmentLength", 
                distances.stream().mapToDouble(Double::doubleValue).average().orElse(0.0));
            metrics.put("shortestSegment", distances.stream().mapToDouble(Double::doubleValue).min().orElse(0.0));
            metrics.put("longestSegment", distances.stream().mapToDouble(Double::doubleValue).max().orElse(0.0));
        }
        
        return metrics;
    }
}