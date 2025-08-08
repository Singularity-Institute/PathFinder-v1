package com.flightplanner;

import com.flightplanner.core.FlightPlanner;
import com.flightplanner.core.FlightPlan;
import com.flightplanner.models.Waypoint;
import com.flightplanner.models.VORStation;
import com.flightplanner.models.Airport;
import com.flightplanner.utils.CoordinateConverter;
import com.flightplanner.utils.CoordinateConverter.MediterraneanData;
import com.flightplanner.utils.FlightAnalyzer;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.DisplayName;
import static org.junit.jupiter.api.Assertions.*;
import java.util.List;

/**
 * Unit tests for the Flight Planner system.
 * Tests core functionality including route planning, optimization, and data handling.
 */
class FlightPlannerTest {
    
    private FlightPlanner planner;
    private Airport tunis;
    private Airport santorini;
    private MediterraneanData testData;
    
    @BeforeEach
    void setUp() {
        planner = new FlightPlanner();
        testData = CoordinateConverter.generateMediterraneanData();
        
        // Add test data to planner
        for (Airport airport : testData.getAirports()) {
            planner.addAirport(airport);
        }
        
        for (VORStation vor : testData.getVors()) {
            planner.addVORStation(vor);
        }
        
        for (Waypoint waypoint : testData.getWaypoints()) {
            planner.addWaypoint(waypoint);
        }
        
        // Get test airports
        tunis = testData.getAirports().stream()
            .filter(a -> "DTTA".equals(a.getIcaoCode()))
            .findFirst()
            .orElseThrow();
            
        santorini = testData.getAirports().stream()
            .filter(a -> "LGSR".equals(a.getIcaoCode()))
            .findFirst()
            .orElseThrow();
    }
    
    @Test
    @DisplayName("Test direct route planning")
    void testDirectRoute() {
        FlightPlan directPlan = planner.planDirectRoute(tunis, santorini);
        
        assertNotNull(directPlan, "Direct plan should not be null");
        assertEquals(2, directPlan.getWaypoints().size(), "Direct route should have 2 waypoints");
        assertEquals(tunis, directPlan.getWaypoints().get(0), "First waypoint should be departure airport");
        assertEquals(santorini, directPlan.getWaypoints().get(1), "Second waypoint should be arrival airport");
        
        double distance = directPlan.getTotalDistance();
        assertTrue(distance > 0, "Distance should be positive");
        assertTrue(distance > 700 && distance < 800, "Distance should be approximately 734.7 nm");
    }
    
    @Test
    @DisplayName("Test optimized route planning")
    void testOptimizedRoute() {
        FlightPlan optimizedPlan = planner.planOptimizedRoute(tunis, santorini);
        
        assertNotNull(optimizedPlan, "Optimized plan should not be null");
        assertTrue(optimizedPlan.getWaypoints().size() >= 2, "Should have at least departure and arrival");
        
        // First and last waypoints should be the airports
        assertEquals(tunis, optimizedPlan.getWaypoints().get(0));
        assertEquals(santorini, optimizedPlan.getWaypoints().get(optimizedPlan.getWaypoints().size() - 1));
        
        double distance = optimizedPlan.getTotalDistance();
        assertTrue(distance > 0, "Distance should be positive");
    }
    
    @Test
    @DisplayName("Test VOR route optimization")
    void testVORRoute() {
        FlightPlanner.VOROptimizedResult result = planner.planVORRoute(tunis, santorini);
        
        assertNotNull(result, "VOR result should not be null");
        assertNotNull(result.getFlightPlan(), "VOR flight plan should not be null");
        assertNotNull(result.getCoverageVORs(), "Coverage VORs list should not be null");
        
        FlightPlan vorPlan = result.getFlightPlan();
        assertTrue(vorPlan.getWaypoints().size() >= 2, "Should have at least departure and arrival");
        assertEquals(tunis, vorPlan.getWaypoints().get(0));
        assertEquals(santorini, vorPlan.getWaypoints().get(vorPlan.getWaypoints().size() - 1));
    }
    
    @Test
    @DisplayName("Test coordinate conversion")
    void testCoordinateConversion() {
        // Test DMS parsing
        double[] coords = CoordinateConverter.parseDMSCoordinate("36°45'30\"N 10°15'45\"E");
        
        assertEquals(2, coords.length, "Should return lat/lon pair");
        assertTrue(coords[0] > 36.7 && coords[0] < 36.8, "Latitude should be approximately 36.7583");
        assertTrue(coords[1] > 10.2 && coords[1] < 10.3, "Longitude should be approximately 10.2625");
        
        // Test conversion back to DMS
        String dmsString = CoordinateConverter.toDMSString(coords[0], coords[1]);
        assertNotNull(dmsString, "DMS string should not be null");
        assertTrue(dmsString.contains("N"), "Should contain North indicator");
        assertTrue(dmsString.contains("E"), "Should contain East indicator");
    }
    
    @Test
    @DisplayName("Test Mediterranean waypoint data")
    void testMediterraneanData() {
        List<Waypoint> waypoints = testData.getWaypoints();
        List<Airport> airports = testData.getAirports();
        List<VORStation> vors = testData.getVors();
        
        assertEquals(14, waypoints.size(), "Should have 14 Mediterranean waypoints");
        assertEquals(2, airports.size(), "Should have 2 airports (Tunis and Santorini)");
        assertEquals(2, vors.size(), "Should have 2 VOR stations (TUC and MIL)");
        
        // Test that all waypoints have valid coordinates
        for (Waypoint wp : waypoints) {
            assertTrue(wp.getLatitude() > 30 && wp.getLatitude() < 45, 
                "Waypoint latitude should be in Mediterranean range");
            assertTrue(wp.getLongitude() > 5 && wp.getLongitude() < 30, 
                "Waypoint longitude should be in Mediterranean range");
        }
    }
    
    @Test
    @DisplayName("Test flight plan analysis")
    void testFlightPlanAnalysis() {
        FlightPlan plan = planner.planDirectRoute(tunis, santorini);
        
        // Test basic metrics
        assertTrue(plan.getTotalDistance() > 0, "Total distance should be positive");
        assertTrue(plan.getFlightTime(450) > 0, "Flight time should be positive");
        assertEquals(1, plan.getSegments().size(), "Direct route should have 1 segment");
        
        // Test analysis report generation
        String report = FlightAnalyzer.generateAnalysisReport(plan, "Test Direct Route");
        assertNotNull(report, "Analysis report should not be null");
        assertTrue(report.contains("FLIGHT PLAN ANALYSIS"), "Report should contain header");
        assertTrue(report.contains("Total Distance"), "Report should contain distance info");
    }
    
    @Test
    @DisplayName("Test flight plan comparison")
    void testFlightPlanComparison() {
        FlightPlan directPlan = planner.planDirectRoute(tunis, santorini);
        FlightPlan optimizedPlan = planner.planOptimizedRoute(tunis, santorini);
        
        String comparison = FlightAnalyzer.compareFlightPlans(
            directPlan, "Direct Route", 
            optimizedPlan, "Optimized Route"
        );
        
        assertNotNull(comparison, "Comparison report should not be null");
        assertTrue(comparison.contains("FLIGHT PLAN COMPARISON"), "Should contain comparison header");
        assertTrue(comparison.contains("DISTANCE COMPARISON"), "Should contain distance comparison");
    }
    
    @Test
    @DisplayName("Test flight plan export formats")
    void testExportFormats() {
        FlightPlan plan = planner.planDirectRoute(tunis, santorini);
        
        // Test JSON export
        String json = plan.exportToJson();
        assertNotNull(json, "JSON export should not be null");
        assertTrue(json.contains("waypoints"), "JSON should contain waypoints");
        assertTrue(json.contains("total_distance"), "JSON should contain total distance");
        
        // Test GPX export
        String gpx = plan.exportToGPX();
        assertNotNull(gpx, "GPX export should not be null");
        assertTrue(gpx.contains("<?xml"), "GPX should be valid XML");
        assertTrue(gpx.contains("<gpx"), "GPX should have gpx element");
        assertTrue(gpx.contains("<rte>"), "GPX should have route element");
    }
    
    @Test
    @DisplayName("Test VOR station functionality")
    void testVORStation() {
        List<VORStation> vors = testData.getVors();
        assertFalse(vors.isEmpty(), "Should have VOR stations");
        
        VORStation tuc = vors.stream()
            .filter(vor -> "TUC".equals(vor.getIdentifier()))
            .findFirst()
            .orElseThrow();
        
        assertEquals("TUC", tuc.getIdentifier(), "VOR identifier should be TUC");
        assertTrue(tuc.getFrequency() > 100, "VOR frequency should be valid");
        assertTrue(tuc.getRange() > 0, "VOR range should be positive");
        
        // Test if VOR is in range of a nearby waypoint
        boolean inRange = tuc.isInRange(tunis);
        assertTrue(inRange, "TUC VOR should be in range of Tunis airport");
    }
    
    @Test
    @DisplayName("Test performance with large waypoint set")
    void testPerformance() {
        // This test ensures the algorithm works with all 14 waypoints
        FlightPlan optimizedPlan = planner.planOptimizedRoute(tunis, santorini);
        
        long startTime = System.currentTimeMillis();
        FlightPlan newPlan = planner.planOptimizedRoute(tunis, santorini);
        long endTime = System.currentTimeMillis();
        
        assertTrue(endTime - startTime < 5000, "Route planning should complete within 5 seconds");
        assertNotNull(newPlan, "Plan should be generated successfully");
    }
}