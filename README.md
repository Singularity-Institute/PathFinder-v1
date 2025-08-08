# Flight Planner Java - Mediterranean Route Optimization

## Overview

This project implements A* pathfinding algorithm with aviation-specific heuristics to optimize flight routes (Example : between Tunis Carthage Airport (DTTA) and Santorini Airport (LGSR), incorporating 14 Mediterranean waypoints and VOR navigation stations).

## Features

### Core Functionality
- **A* Pathfinding Algorithm**: Aviation-optimized with VOR preferences and segment penalties
- **VOR Navigation**: TUC and MIL VOR stations with 10% cost reduction
- **Mediterranean Route**: Tunis (DTTA) to Santorini (LGSR) with 14 intermediate waypoints
- **DMS Coordinate Conversion**: Parse coordinates in Degrees Minutes Seconds format
- **Multiple Export Formats**: JSON, GPX, KML for various applications

### Optimization Features
- **VOR Preference**: 10% cost reduction for routes using VOR stations
- **Long Segment Penalty**: 20% penalty for segments exceeding 200nm
- **Waypoint Sequence Optimization**: TSP-based optimization for multiple waypoints
- **Route Analysis**: Comprehensive flight plan analysis and comparison

### Technical Features
- **Maven Build System**: Standard Java project structure
- **JUnit Testing**: Comprehensive test suite
- **Modular Architecture**: Clean separation of concerns
- **Export Capabilities**: Multiple file formats for integration

## Project Structure

```
flight_planner_java/
├── pom.xml                           # Maven configuration
├── src/main/java/com/flightplanner/
│   ├── Main.java                     # Application entry point
│   ├── models/                       # Core data models
│   │   ├── Waypoint.java            # Base waypoint class
│   │   ├── VORStation.java          # VOR navigation station
│   │   ├── Airport.java             # Airport with runways
│   │   └── Runway.java              # Airport runway data
│   ├── algorithms/                   # Pathfinding algorithms
│   │   ├── PathOptimizer.java       # A* implementation
│   │   └── AStarNode.java           # A* node representation
│   ├── core/                        # Core flight planning
│   │   ├── FlightPlanner.java       # Main planner class
│   │   └── FlightPlan.java          # Flight plan representation
│   ├── utils/                       # Utilities and helpers
│   │   ├── CoordinateConverter.java # DMS conversion and data
│   │   ├── FlightAnalyzer.java      # Analysis and reporting
│   │   └── FlightVisualizer.java    # Visualization tools
│   └── examples/
│       └── MediterraneanRouteExample.java  # Complete example
└── src/test/java/com/flightplanner/
    └── FlightPlannerTest.java       # JUnit test suite
```

## Mediterranean Route Data

### Airports
- **Tunis Carthage (DTTA/TUN)**: 36.851°N, 10.227°E
- **Santorini (LGSR/JTR)**: 36.399°N, 25.479°E

### VOR Stations
- **TUC VOR**: 36.8°N, 10.2°E (115.5 MHz, 300nm range)
- **MIL VOR**: 37.5°N, 15.0°E (116.2 MHz, 300nm range)

### Waypoints (DMS Converted)
14 Mediterranean waypoints (WP1-WP14) converted from DMS coordinates:
- WP1: 36°45'30"N 10°15'45"E → 36.7583°N, 10.2625°E
- WP2: 37°12'15"N 11°30'20"E → 37.2042°N, 11.5056°E
- ... (complete list in CoordinateConverter.java)

## Algorithm Details

### A* Pathfinding
- **Heuristic**: Haversine distance (great circle)
- **Cost Function**: Base distance with aviation modifiers
- **VOR Bonus**: 10% cost reduction (multiplier 0.9)
- **Long Segment Penalty**: 20% increase for segments > 200nm
- **Range Limit**: Maximum 500nm between waypoints

### Cost Calculation
```java
cost = baseDistance;
if (useVorPreferred && destination instanceof VORStation) {
    cost *= 0.9;  // 10% VOR discount
}
if (baseDistance > 200nm) {
    cost *= 1.2;  // 20% long segment penalty  
}
```

## Usage Examples

### Basic Route Planning
```java
FlightPlanner planner = new FlightPlanner();
MediterraneanData data = CoordinateConverter.generateMediterraneanData();

// Add data to planner
data.getAirports().forEach(planner::addAirport);
data.getVors().forEach(planner::addVORStation);
data.getWaypoints().forEach(planner::addWaypoint);

// Plan routes
FlightPlan direct = planner.planDirectRoute(tunis, santorini);
FlightPlan optimized = planner.planOptimizedRoute(tunis, santorini);
```

### VOR Route Optimization
```java
FlightPlanner.VOROptimizedResult result = planner.planVORRoute(tunis, santorini);
FlightPlan vorPlan = result.getFlightPlan();
List<VORStation> coverage = result.getCoverageVORs();
```

### Analysis and Export
```java
String analysis = FlightAnalyzer.generateAnalysisReport(plan, "Route Analysis");
planner.exportFlightPlan(plan, "route.json", "json");
planner.exportFlightPlan(plan, "route.gpx", "gpx");
```

## Build and Test

### Prerequisites
- Java 11 or higher
- Maven 3.6+

### Building
```bash
mvn clean compile
mvn test
mvn package
```

### Running
```bash
java -jar target/flight-planner-1.0.jar
java -jar target/flight-planner-1.0.jar help
```

## Expected Results

Based on the Python version testing:
- **Direct Route**: ~734.7 nm (2 waypoints)
- **Optimized Route**: ~750.9 nm (multiple waypoints with VOR preference)
- **VOR Coverage**: Route includes TUC and MIL VOR stations for navigation
- **Export**: JSON, GPX, and KML files generated in `output/` directory

## Code Statistics

- **Total Files**: 14 Java files
- **Lines of Code**: 2,534 lines
- **Test Coverage**: Comprehensive JUnit tests
- **Dependencies**: Minimal (Gson for JSON, JUnit for testing)

## Comparison with Python Version

| Feature | Python | Java | Status |
|---------|--------|------|--------|
| A* Algorithm | ✓ | ✓ | Complete |
| Mediterranean Data | ✓ | ✓ | Complete |
| DMS Conversion | ✓ | ✓ | Complete |
| VOR Optimization | ✓ | ✓ | Complete |
| Export Formats | ✓ | ✓ | Complete |
| Analysis Tools | ✓ | ✓ | Complete |
| Visualization | ✓ | ✓ | Complete |
| Testing | ✓ | ✓ | Complete |

## Architecture Benefits

- **Type Safety**: Strong typing prevents runtime errors
- **Performance**: Compiled bytecode for better execution speed
- **Maintainability**: Clear class hierarchy and separation of concerns
- **Extensibility**: Plugin architecture for new algorithms and data sources
- **Integration**: Easy integration with Java enterprise systems

## License

Same licensing as the original Python project. Educational and research purposes.

---

*This Java implementation provides complete feature parity with the Python version while leveraging Java's strengths in type safety, performance, and enterprise integration.*
