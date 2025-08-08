package com.flightplanner.utils;

import com.flightplanner.core.FlightPlan;
import com.flightplanner.models.Waypoint;
import com.flightplanner.models.VORStation;
import com.flightplanner.models.Airport;
import java.util.List;
import java.io.FileWriter;
import java.io.IOException;

/**
 * Utility class for visualizing flight plans and generating visual outputs.
 * Creates text-based visualizations and export formats for analysis.
 */
public class FlightVisualizer {
    
    /**
     * Generate ASCII map representation of a flight plan.
     * Creates a simple text-based visualization of the route.
     * 
     * @param plan Flight plan to visualize
     * @param width Width of the ASCII map
     * @param height Height of the ASCII map
     * @return ASCII map as string
     */
    public static String generateASCIIMap(FlightPlan plan, int width, int height) {
        List<Waypoint> waypoints = plan.getWaypoints();
        if (waypoints.isEmpty()) {
            return "No waypoints to display";
        }
        
        // Find bounds
        double minLat = waypoints.stream().mapToDouble(Waypoint::getLatitude).min().orElse(0);
        double maxLat = waypoints.stream().mapToDouble(Waypoint::getLatitude).max().orElse(0);
        double minLon = waypoints.stream().mapToDouble(Waypoint::getLongitude).min().orElse(0);
        double maxLon = waypoints.stream().mapToDouble(Waypoint::getLongitude).max().orElse(0);
        
        // Add padding
        double latPadding = (maxLat - minLat) * 0.1;
        double lonPadding = (maxLon - minLon) * 0.1;
        minLat -= latPadding;
        maxLat += latPadding;
        minLon -= lonPadding;
        maxLon += lonPadding;
        
        // Create map grid
        char[][] map = new char[height][width];
        for (int i = 0; i < height; i++) {
            for (int j = 0; j < width; j++) {
                map[i][j] = ' ';
            }
        }
        
        // Plot waypoints and route
        for (int i = 0; i < waypoints.size(); i++) {
            Waypoint wp = waypoints.get(i);
            int x = (int) ((wp.getLongitude() - minLon) / (maxLon - minLon) * (width - 1));
            int y = (int) ((maxLat - wp.getLatitude()) / (maxLat - minLat) * (height - 1));
            
            if (x >= 0 && x < width && y >= 0 && y < height) {
                if (wp instanceof Airport) {
                    map[y][x] = (i == 0) ? 'S' : 'E'; // Start/End airports
                } else if (wp instanceof VORStation) {
                    map[y][x] = 'V'; // VOR stations
                } else {
                    map[y][x] = '*'; // Regular waypoints
                }
            }
            
            // Draw line to next waypoint
            if (i < waypoints.size() - 1) {
                drawLine(map, waypoints.get(i), waypoints.get(i + 1), 
                        minLat, maxLat, minLon, maxLon, width, height);
            }
        }
        
        // Convert to string
        StringBuilder result = new StringBuilder();
        result.append("Flight Route Visualization (").append(width).append("x").append(height).append(")\n");
        result.append("Legend: S=Start, E=End, V=VOR, *=Waypoint, -=Route\n");
        result.append("Bounds: ").append(String.format("%.2f°-%.2f°N, %.2f°-%.2f°E", minLat, maxLat, minLon, maxLon)).append("\n\n");
        
        for (int i = 0; i < height; i++) {
            for (int j = 0; j < width; j++) {
                result.append(map[i][j]);
            }
            result.append('\n');
        }
        
        return result.toString();
    }
    
    /**
     * Draw a line between two waypoints on the ASCII map.
     */
    private static void drawLine(char[][] map, Waypoint wp1, Waypoint wp2,
                                double minLat, double maxLat, double minLon, double maxLon,
                                int width, int height) {
        int x1 = (int) ((wp1.getLongitude() - minLon) / (maxLon - minLon) * (width - 1));
        int y1 = (int) ((maxLat - wp1.getLatitude()) / (maxLat - minLat) * (height - 1));
        int x2 = (int) ((wp2.getLongitude() - minLon) / (maxLon - minLon) * (width - 1));
        int y2 = (int) ((maxLat - wp2.getLatitude()) / (maxLat - minLat) * (height - 1));
        
        // Simple line drawing using Bresenham-like approach
        int dx = Math.abs(x2 - x1);
        int dy = Math.abs(y2 - y1);
        int sx = x1 < x2 ? 1 : -1;
        int sy = y1 < y2 ? 1 : -1;
        int err = dx - dy;
        
        int x = x1;
        int y = y1;
        
        while (true) {
            if (x >= 0 && x < width && y >= 0 && y < height && map[y][x] == ' ') {
                map[y][x] = '-';
            }
            
            if (x == x2 && y == y2) break;
            
            int e2 = 2 * err;
            if (e2 > -dy) {
                err -= dy;
                x += sx;
            }
            if (e2 < dx) {
                err += dx;
                y += sy;
            }
        }
    }
    
    /**
     * Generate a detailed route table for a flight plan.
     * 
     * @param plan Flight plan to tabulate
     * @return Formatted route table as string
     */
    public static String generateRouteTable(FlightPlan plan) {
        StringBuilder table = new StringBuilder();
        
        // Header
        table.append("FLIGHT ROUTE TABLE\n");
        table.append("=".repeat(80)).append("\n");
        table.append(String.format("%-4s %-20s %-12s %-12s %-8s %-8s %-8s%n", 
            "LEG", "WAYPOINT", "LATITUDE", "LONGITUDE", "DIST", "BEARING", "TYPE"));
        table.append("-".repeat(80)).append("\n");
        
        List<Waypoint> waypoints = plan.getWaypoints();
        List<FlightPlan.Segment> segments = plan.getSegments();
        
        // First waypoint
        if (!waypoints.isEmpty()) {
            Waypoint first = waypoints.get(0);
            table.append(String.format("%-4s %-20s %11.6f° %12.6f° %-8s %-8s %-8s%n", 
                "DEP", truncate(first.getName(), 20), first.getLatitude(), first.getLongitude(), 
                "---", "---", first.getWaypointType()));
        }
        
        // Route segments
        double cumulativeDistance = 0;
        for (int i = 0; i < segments.size(); i++) {
            FlightPlan.Segment segment = segments.get(i);
            cumulativeDistance += segment.getDistance();
            Waypoint to = segment.getTo();
            
            table.append(String.format("%-4d %-20s %11.6f° %12.6f° %7.1fnm %7.0f° %-8s%n", 
                i + 1, truncate(to.getName(), 20), to.getLatitude(), to.getLongitude(), 
                segment.getDistance(), segment.getBearing(), segment.getLegType()));
        }
        
        // Summary
        table.append("-".repeat(80)).append("\n");
        table.append(String.format("TOTAL DISTANCE: %.1f nm\n", plan.getTotalDistance()));
        table.append(String.format("TOTAL WAYPOINTS: %d\n", waypoints.size()));
        
        return table.toString();
    }
    
    /**
     * Generate KML file content for Google Earth visualization.
     * 
     * @param plan Flight plan to export
     * @param planName Name for the flight plan
     * @return KML content as string
     */
    public static String generateKML(FlightPlan plan, String planName) {
        StringBuilder kml = new StringBuilder();
        
        kml.append("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
        kml.append("<kml xmlns=\"http://www.opengis.net/kml/2.2\">\n");
        kml.append("  <Document>\n");
        kml.append("    <name>").append(planName).append("</name>\n");
        kml.append("    <description>Flight Plan Route</description>\n");
        
        // Style definitions
        kml.append("    <Style id=\"routeStyle\">\n");
        kml.append("      <LineStyle>\n");
        kml.append("        <color>ff0000ff</color>\n"); // Red line
        kml.append("        <width>3</width>\n");
        kml.append("      </LineStyle>\n");
        kml.append("    </Style>\n");
        
        kml.append("    <Style id=\"waypointStyle\">\n");
        kml.append("      <IconStyle>\n");
        kml.append("        <color>ff00ff00</color>\n"); // Green icons
        kml.append("        <scale>1.2</scale>\n");
        kml.append("      </IconStyle>\n");
        kml.append("    </Style>\n");
        
        // Waypoint placemarks
        List<Waypoint> waypoints = plan.getWaypoints();
        for (int i = 0; i < waypoints.size(); i++) {
            Waypoint wp = waypoints.get(i);
            kml.append("    <Placemark>\n");
            kml.append("      <name>").append(wp.getName()).append("</name>\n");
            kml.append("      <description>").append(wp.getWaypointType()).append("</description>\n");
            kml.append("      <styleUrl>#waypointStyle</styleUrl>\n");
            kml.append("      <Point>\n");
            kml.append("        <coordinates>").append(wp.getLongitude()).append(",")
               .append(wp.getLatitude()).append(",").append(wp.getAltitude() != null ? wp.getAltitude() : 0)
               .append("</coordinates>\n");
            kml.append("      </Point>\n");
            kml.append("    </Placemark>\n");
        }
        
        // Route line
        kml.append("    <Placemark>\n");
        kml.append("      <name>Flight Route</name>\n");
        kml.append("      <description>Optimized flight path</description>\n");
        kml.append("      <styleUrl>#routeStyle</styleUrl>\n");
        kml.append("      <LineString>\n");
        kml.append("        <tessellate>1</tessellate>\n");
        kml.append("        <coordinates>\n");
        
        for (Waypoint wp : waypoints) {
            kml.append("          ").append(wp.getLongitude()).append(",")
               .append(wp.getLatitude()).append(",").append(wp.getAltitude() != null ? wp.getAltitude() : 35000)
               .append("\n");
        }
        
        kml.append("        </coordinates>\n");
        kml.append("      </LineString>\n");
        kml.append("    </Placemark>\n");
        
        kml.append("  </Document>\n");
        kml.append("</kml>\n");
        
        return kml.toString();
    }
    
    /**
     * Export visualization to file.
     * 
     * @param content Content to export
     * @param filename Output filename
     * @throws IOException If file writing fails
     */
    public static void exportToFile(String content, String filename) throws IOException {
        try (FileWriter writer = new FileWriter(filename)) {
            writer.write(content);
        }
    }
    
    /**
     * Generate comprehensive visualization report.
     * 
     * @param plan Flight plan to visualize
     * @param planName Name of the plan
     * @return Complete visualization report
     */
    public static String generateVisualizationReport(FlightPlan plan, String planName) {
        StringBuilder report = new StringBuilder();
        
        report.append("FLIGHT PLAN VISUALIZATION REPORT\n");
        report.append("=".repeat(50)).append("\n\n");
        report.append("Plan Name: ").append(planName).append("\n\n");
        
        // ASCII Map
        report.append("ASCII ROUTE MAP:\n");
        report.append("-".repeat(20)).append("\n");
        report.append(generateASCIIMap(plan, 60, 20)).append("\n\n");
        
        // Route Table
        report.append(generateRouteTable(plan)).append("\n\n");
        
        // Analysis summary
        report.append("VISUALIZATION SUMMARY:\n");
        report.append("-".repeat(22)).append("\n");
        report.append("Available export formats:\n");
        report.append("  - ASCII Map (text-based visualization)\n");
        report.append("  - Route Table (detailed waypoint listing)\n");
        report.append("  - KML (Google Earth compatible)\n");
        report.append("  - GPX (GPS device compatible)\n");
        report.append("  - JSON (programmatic access)\n");
        
        return report.toString();
    }
    
    /**
     * Truncate string to specified length.
     */
    private static String truncate(String str, int maxLength) {
        if (str.length() <= maxLength) {
            return str;
        }
        return str.substring(0, maxLength - 3) + "...";
    }
}