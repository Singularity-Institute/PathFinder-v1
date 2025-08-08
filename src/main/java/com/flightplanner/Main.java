package com.flightplanner;

import com.flightplanner.examples.MediterraneanRouteExample;

/**
 * Main entry point for the Flight Planner Java application.
 * Demonstrates the Mediterranean route planning functionality.
 */
public class Main {
    
    public static void main(String[] args) {
        System.out.println("Flight Planner Java - Mediterranean Route Optimization");
        System.out.println("=====================================================");
        System.out.println();
        
        if (args.length > 0 && "help".equals(args[0])) {
            printHelp();
            return;
        }
        
        try {
            // Run the Mediterranean route example
            MediterraneanRouteExample.main(args);
            
        } catch (Exception e) {
            System.err.println("Error running flight planner: " + e.getMessage());
            e.printStackTrace();
            System.exit(1);
        }
        
        System.out.println();
        System.out.println("Flight planning completed successfully!");
        System.out.println("Check the output directory for exported flight plans.");
    }
    
    private static void printHelp() {
        System.out.println("Flight Planner Java - Help");
        System.out.println("==========================");
        System.out.println();
        System.out.println("This application demonstrates flight path optimization between");
        System.out.println("Tunis Carthage Airport (DTTA) and Santorini Airport (LGSR).");
        System.out.println();
        System.out.println("Features:");
        System.out.println("  - A* pathfinding algorithm with aviation-specific heuristics");
        System.out.println("  - VOR station preference with 10% cost reduction");
        System.out.println("  - Long segment penalties (20% for segments > 200nm)");
        System.out.println("  - 14 Mediterranean waypoints from DMS coordinates");
        System.out.println("  - TUC and MIL VOR stations for navigation");
        System.out.println("  - Multiple export formats (JSON, GPX, KML)");
        System.out.println();
        System.out.println("Usage:");
        System.out.println("  java -jar flight-planner.jar        # Run Mediterranean example");
        System.out.println("  java -jar flight-planner.jar help   # Show this help");
        System.out.println();
        System.out.println("Output files are saved to: output/");
    }
}