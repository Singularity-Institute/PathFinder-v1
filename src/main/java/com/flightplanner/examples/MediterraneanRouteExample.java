package com.flightplanner.examples;

import com.flightplanner.core.FlightPlanner;
import com.flightplanner.core.FlightPlan;
import com.flightplanner.models.Waypoint;
import com.flightplanner.models.VORStation;
import com.flightplanner.models.Airport;
import com.flightplanner.utils.CoordinateConverter;
import com.flightplanner.utils.CoordinateConverter.MediterraneanData;
import java.util.List;
import java.io.IOException;

/**
 * Example demonstrating Mediterranean route planning from Tunis to Santorini.
 * This example recreates the Python version's Mediterranean route optimization.
 */
public class MediterraneanRouteExample {
    
    public static void main(String[] args) {
        System.out.println("=== Mediterranean Flight Route Planning ===");
        System.out.println("Route: Tunis Carthage (DTTA) → Santorini (LGSR)");
        System.out.println();
        
        try {
            // Create flight planner
            FlightPlanner planner = new FlightPlanner();
            
            // Load Mediterranean data
            MediterraneanData data = CoordinateConverter.generateMediterraneanData();
            
            // Add data to planner
            System.out.println("Loading Mediterranean navigation data...");
            for (Airport airport : data.getAirports()) {
                planner.addAirport(airport);
                System.out.println("  Added airport: " + airport);
            }
            
            for (VORStation vor : data.getVors()) {
                planner.addVORStation(vor);
                System.out.println("  Added VOR: " + vor);
            }
            
            for (Waypoint waypoint : data.getWaypoints()) {
                planner.addWaypoint(waypoint);
            }
            System.out.println("  Added " + data.getWaypoints().size() + " waypoints (WP1-WP14)");
            System.out.println();
            
            // Get airports
            Airport tunis = data.getAirports().stream()
                .filter(a -> "DTTA".equals(a.getIcaoCode()))
                .findFirst()
                .orElseThrow(() -> new RuntimeException("Tunis airport not found"));
                
            Airport santorini = data.getAirports().stream()
                .filter(a -> "LGSR".equals(a.getIcaoCode()))
                .findFirst()
                .orElseThrow(() -> new RuntimeException("Santorini airport not found"));
            
            // Plan direct route
            System.out.println("1. DIRECT ROUTE ANALYSIS");
            System.out.println("========================");
            FlightPlan directPlan = planner.planDirectRoute(tunis, santorini);
            analyzeFlightPlan(directPlan, "Direct Route");
            
            // Plan optimized route
            System.out.println("\n2. OPTIMIZED ROUTE WITH VOR PREFERENCE");
            System.out.println("======================================");
            FlightPlan optimizedPlan = planner.planOptimizedRoute(tunis, santorini);
            analyzeFlightPlan(optimizedPlan, "Optimized Route");
            
            // Plan VOR-specific route
            System.out.println("\n3. VOR-OPTIMIZED ROUTE ANALYSIS");
            System.out.println("===============================");
            FlightPlanner.VOROptimizedResult vorResult = planner.planVORRoute(tunis, santorini);
            FlightPlan vorPlan = vorResult.getFlightPlan();
            analyzeFlightPlan(vorPlan, "VOR-Optimized Route");
            
            // Show VOR coverage
            System.out.println("\nVOR Coverage Analysis:");
            List<VORStation> coverageVORs = vorResult.getCoverageVORs();
            if (coverageVORs.isEmpty()) {
                System.out.println("  No VOR stations provide coverage for this route");
            } else {
                for (VORStation vor : coverageVORs) {
                    System.out.println("  " + vor + " provides navigation coverage");
                }
            }
            
            // Compare routes
            System.out.println("\n4. ROUTE COMPARISON");
            System.out.println("==================");
            compareRoutes(directPlan, optimizedPlan, vorPlan);
            
            // Export plans
            System.out.println("\n5. EXPORTING FLIGHT PLANS");
            System.out.println("=========================");
            exportFlightPlans(planner, directPlan, optimizedPlan, vorPlan);
            
            // Show waypoint details
            System.out.println("\n6. MEDITERRANEAN WAYPOINTS DETAILS");
            System.out.println("==================================");
            showWaypointDetails(data.getWaypoints());
            
        } catch (Exception e) {
            System.err.println("Error: " + e.getMessage());
            e.printStackTrace();
        }
    }
    
    /**
     * Analyze and display flight plan details.
     */
    private static void analyzeFlightPlan(FlightPlan plan, String routeName) {
        System.out.println(routeName + ":");
        System.out.printf("  Total Distance: %.1f nm%n", plan.getTotalDistance());
        System.out.printf("  Waypoints: %d%n", plan.getWaypoints().size());
        System.out.printf("  Estimated Flight Time (450 kts): %.1f hours%n", plan.getFlightTime(450));
        
        System.out.println("  Route:");
        List<Waypoint> waypoints = plan.getWaypoints();
        for (int i = 0; i < waypoints.size(); i++) {
            Waypoint wp = waypoints.get(i);
            String prefix = (i == 0) ? "    " : "    → ";
            System.out.printf("%s%s (%.3f°, %.3f°)%n", 
                prefix, wp.getName(), wp.getLatitude(), wp.getLongitude());
        }
        
        System.out.println("  Segments:");
        for (FlightPlan.Segment segment : plan.getSegments()) {
            System.out.printf("    %s%n", segment);
        }
    }
    
    /**
     * Compare different route options.
     */
    private static void compareRoutes(FlightPlan direct, FlightPlan optimized, FlightPlan vor) {
        System.out.printf("Route Comparison:%n");
        System.out.printf("  Direct Route:     %.1f nm (%d waypoints)%n", 
            direct.getTotalDistance(), direct.getWaypoints().size());
        System.out.printf("  Optimized Route:  %.1f nm (%d waypoints)%n", 
            optimized.getTotalDistance(), optimized.getWaypoints().size());
        System.out.printf("  VOR Route:        %.1f nm (%d waypoints)%n", 
            vor.getTotalDistance(), vor.getWaypoints().size());
        
        double directDist = direct.getTotalDistance();
        double optimizedDiff = optimized.getTotalDistance() - directDist;
        double vorDiff = vor.getTotalDistance() - directDist;
        
        System.out.printf("%nDistance Differences:%n");
        System.out.printf("  Optimized vs Direct: %+.1f nm (%.1f%%)%n", 
            optimizedDiff, (optimizedDiff / directDist) * 100);
        System.out.printf("  VOR vs Direct:       %+.1f nm (%.1f%%)%n", 
            vorDiff, (vorDiff / directDist) * 100);
    }
    
    /**
     * Export flight plans to files.
     */
    private static void exportFlightPlans(FlightPlanner planner, FlightPlan direct, 
                                        FlightPlan optimized, FlightPlan vor) {
        try {
            String baseDir = "/home/makoudhai/flight_planner_java/output/";
            
            // Create output directory if it doesn't exist
            java.io.File outputDir = new java.io.File(baseDir);
            outputDir.mkdirs();
            
            // Export JSON files
            planner.exportFlightPlan(direct, baseDir + "mediterranean_direct_route.json", "json");
            planner.exportFlightPlan(optimized, baseDir + "mediterranean_optimized_route.json", "json");
            planner.exportFlightPlan(vor, baseDir + "mediterranean_vor_route.json", "json");
            
            // Export GPX files
            planner.exportFlightPlan(direct, baseDir + "mediterranean_direct_route.gpx", "gpx");
            planner.exportFlightPlan(optimized, baseDir + "mediterranean_optimized_route.gpx", "gpx");
            planner.exportFlightPlan(vor, baseDir + "mediterranean_vor_route.gpx", "gpx");
            
            System.out.println("  Exported flight plans to " + baseDir);
            System.out.println("    - JSON format: *_route.json");
            System.out.println("    - GPX format: *_route.gpx");
            
        } catch (IOException e) {
            System.err.println("  Export failed: " + e.getMessage());
        }
    }
    
    /**
     * Show details of Mediterranean waypoints.
     */
    private static void showWaypointDetails(List<Waypoint> waypoints) {
        System.out.println("Mediterranean Waypoints (converted from DMS):");
        for (Waypoint wp : waypoints) {
            String dmsCoords = CoordinateConverter.toDMSString(wp.getLatitude(), wp.getLongitude());
            System.out.printf("  %s: %.6f°, %.6f° (%s)%n", 
                wp.getName(), wp.getLatitude(), wp.getLongitude(), dmsCoords);
        }
    }
}