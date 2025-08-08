# Diagrammes de Classes - Flight Planner Java

## Table des Matières

1. [Diagramme de classes principal](#diagramme-de-classes-principal)
2. [Diagramme de packages](#diagramme-de-packages)
3. [Diagrammes de séquence](#diagrammes-de-séquence)
4. [Diagrammes d'états](#diagrammes-détats)
5. [Relations et cardinalités](#relations-et-cardinalités)

---

## Diagramme de classes principal

### Vue d'ensemble architecturale

```mermaid
classDiagram
    %% Classe de base Waypoint
    class Waypoint {
        #String name
        #double latitude
        #double longitude
        #Double altitude
        #String waypointType
        #static double EARTH_RADIUS_NM
        +Waypoint(String, double, double, Double, String)
        +distanceTo(Waypoint) double
        +bearingTo(Waypoint) double
        +equals(Object) boolean
        +hashCode() int
        +toString() String
        +getName() String
        +getLatitude() double
        +getLongitude() double
        +getAltitude() Double
        +getWaypointType() String
    }

    %% Classe Airport héritant de Waypoint
    class Airport {
        -String icaoCode
        -String iataCode
        -String airportType
        -double elevationFt
        -List~Runway~ runways
        +Airport(String, double, double, String, String)
        +Airport(String, double, double, String, String, String, double, List~Runway~)
        +addRunway(Runway) void
        +getLongestRunway() Optional~Runway~
        +canAccommodateAircraft(double) boolean
        +getRunwaysWithMinLength(double) List~Runway~
        +getIcaoCode() String
        +getIataCode() String
        +getAirportType() String
        +getElevationFt() double
        +getRunways() List~Runway~
    }

    %% Classe VORStation héritant de Waypoint
    class VORStation {
        -double frequency
        -String identifier
        -double magneticVariation
        -double rangeNm
        -boolean dmeAvailable
        +VORStation(String, double, double, double, String, Double, double, double, boolean)
        +VORStation(String, double, double, double, String)
        +isInRange(Waypoint) boolean
        +getBearing(Waypoint) double
        +getFrequency() double
        +getIdentifier() String
        +getMagneticVariation() double
        +getRange() double
        +isDmeAvailable() boolean
    }

    %% Classe Runway
    class Runway {
        -String designation
        -double lengthFt
        -double widthFt
        -String surfaceType
        -double headingMagnetic
        -boolean lightingAvailable
        +Runway(String, double, double, String, double, boolean)
        +Runway(String, double, double)
        +getDesignation() String
        +getLengthFt() double
        +getWidthFt() double
        +getSurfaceType() String
        +getHeadingMagnetic() double
        +isLightingAvailable() boolean
    }

    %% Classe principale FlightPlanner
    class FlightPlanner {
        -List~Airport~ airports
        -List~VORStation~ vorStations
        -List~Waypoint~ waypoints
        -PathOptimizer optimizer
        +FlightPlanner()
        +addAirport(Airport) void
        +addVORStation(VORStation) void
        +addWaypoint(Waypoint) void
        +planDirectRoute(Airport, Airport) FlightPlan
        +planOptimizedRoute(Airport, Airport, List~Waypoint~, boolean) FlightPlan
        +planOptimizedRoute(Airport, Airport) FlightPlan
        +planVORRoute(Airport, Airport) VOROptimizedResult
        +findAlternateAirports(Airport, double) List~AlternateAirport~
        +getVORCoverage(Waypoint) List~VORStation~
        +exportFlightPlan(FlightPlan, String, String) void
        -updateOptimizer() void
    }

    %% Classe PathOptimizer pour algorithme A*
    class PathOptimizer {
        -List~Waypoint~ waypoints
        -List~VORStation~ vorStations
        -List~Waypoint~ allPoints
        -static double MAX_RANGE_NM
        -static double VOR_COST_MULTIPLIER
        -static double LONG_SEGMENT_PENALTY
        -static double LONG_SEGMENT_THRESHOLD
        +PathOptimizer(List~Waypoint~, List~VORStation~)
        +findOptimalPath(Waypoint, Waypoint, boolean) List~Waypoint~
        +optimizeWaypointSequence(List~Waypoint~) List~Waypoint~
        +findVORRoute(Waypoint, Waypoint) PathWithVORCoverage
        -aStar(Waypoint, Waypoint, boolean) List~Waypoint~
        -getNeighbors(Waypoint) List~Waypoint~
        -calculateCost(Waypoint, Waypoint, boolean) double
        -heuristic(Waypoint, Waypoint) double
        -reconstructPath(AStarNode) List~Waypoint~
        -nearestNeighborTSP(List~Waypoint~) List~Waypoint~
    }

    %% Classe AStarNode pour l'algorithme
    class AStarNode {
        -Waypoint waypoint
        -double gScore
        -double fScore
        -AStarNode parent
        +AStarNode(Waypoint)
        +AStarNode(Waypoint, double, double)
        +compareTo(AStarNode) int
        +equals(Object) boolean
        +hashCode() int
        +getWaypoint() Waypoint
        +getGScore() double
        +setGScore(double) void
        +getFScore() double
        +setFScore(double) void
        +getParent() AStarNode
        +setParent(AStarNode) void
    }

    %% Classe FlightPlan pour représenter le résultat
    class FlightPlan {
        -List~Waypoint~ waypoints
        -double totalDistance
        -List~Segment~ segments
        +FlightPlan(List~Waypoint~)
        +getFlightTime(double) double
        +exportToMap() Map~String, Object~
        +exportToJson() String
        +exportToGPX() String
        +getWaypoints() List~Waypoint~
        +getTotalDistance() double
        +getSegments() List~Segment~
        -calculateTotalDistance() double
        -createSegments() List~Segment~
        -determineLegType(Waypoint, Waypoint) String
    }

    %% Classe Segment interne à FlightPlan
    class Segment {
        -Waypoint from
        -Waypoint to
        -double distance
        -double bearing
        -String legType
        +Segment(Waypoint, Waypoint, double, double, String)
        +getFrom() Waypoint
        +getTo() Waypoint
        +getDistance() double
        +getBearing() double
        +getLegType() String
        +toString() String
    }

    %% Classes utilitaires
    class CoordinateConverter {
        -static Pattern DMS_PATTERN
        +parseDMSCoordinate(String) double[]
        +createMediterraneanAirports() List~Airport~
        +createMediterraneanVORs() List~VORStation~
        +createMediterraneanWaypoints() List~Waypoint~
        +toDMSString(double, double) String
        +generateMediterraneanData() MediterraneanData
        -formatDMS(double, boolean) String
    }

    class FlightAnalyzer {
        +generateAnalysisReport(FlightPlan, String) String
        +compareFlightPlans(FlightPlan, String, FlightPlan, String) String
        +estimateFuelConsumption(FlightPlan, double, double) double
        +calculatePerformanceMetrics(FlightPlan) Map~String, Double~
        -appendBasicStatistics(StringBuilder, FlightPlan) void
        -appendRouteDetails(StringBuilder, FlightPlan) void
        -appendSegmentAnalysis(StringBuilder, FlightPlan) void
        -appendNavigationAnalysis(StringBuilder, FlightPlan) void
    }

    class FlightVisualizer {
        +generateASCIIMap(FlightPlan, int, int) String
        +generateRouteTable(FlightPlan) String
        +generateKML(FlightPlan, String) String
        +exportToFile(String, String) void
        +generateVisualizationReport(FlightPlan, String) String
        -drawLine(char[][], Waypoint, Waypoint, double, double, double, double, int, int) void
        -truncate(String, int) String
    }

    %% Classes de résultat
    class VOROptimizedResult {
        -FlightPlan flightPlan
        -List~VORStation~ coverageVORs
        +VOROptimizedResult(FlightPlan, List~VORStation~)
        +getFlightPlan() FlightPlan
        +getCoverageVORs() List~VORStation~
    }

    class AlternateAirport {
        -Airport airport
        -double distance
        +AlternateAirport(Airport, double)
        +getAirport() Airport
        +getDistance() double
        +toString() String
    }

    class PathWithVORCoverage {
        -List~Waypoint~ path
        -List~VORStation~ coverageVORs
        +PathWithVORCoverage(List~Waypoint~, List~VORStation~)
        +getPath() List~Waypoint~
        +getCoverageVORs() List~VORStation~
    }

    class MediterraneanData {
        -List~Waypoint~ waypoints
        -List~Airport~ airports
        -List~VORStation~ vors
        +MediterraneanData(List~Waypoint~, List~Airport~, List~VORStation~)
        +getWaypoints() List~Waypoint~
        +getAirports() List~Airport~
        +getVors() List~VORStation~
    }

    %% Relations d'héritage
    Waypoint <|-- Airport
    Waypoint <|-- VORStation

    %% Relations de composition et agrégation
    Airport *-- Runway : contains
    FlightPlanner *-- PathOptimizer : uses
    FlightPlanner o-- Airport : manages
    FlightPlanner o-- VORStation : manages
    FlightPlanner o-- Waypoint : manages
    PathOptimizer --> AStarNode : creates
    PathOptimizer --> PathWithVORCoverage : returns
    FlightPlanner --> FlightPlan : creates
    FlightPlanner --> VOROptimizedResult : creates
    FlightPlanner --> AlternateAirport : creates
    FlightPlan *-- Segment : contains
    FlightPlan o-- Waypoint : references
    CoordinateConverter --> MediterraneanData : creates
    VOROptimizedResult *-- FlightPlan : contains
    VOROptimizedResult o-- VORStation : references
    PathWithVORCoverage o-- Waypoint : references
    PathWithVORCoverage o-- VORStation : references
    AlternateAirport *-- Airport : contains
```

---

## Diagramme de packages

### Structure des packages

```mermaid
graph TB
    subgraph "com.flightplanner"
        Main[Main.java]
    end

    subgraph "com.flightplanner.models"
        Waypoint[Waypoint.java]
        Airport[Airport.java]
        VORStation[VORStation.java]
        Runway[Runway.java]
    end

    subgraph "com.flightplanner.core"
        FlightPlanner[FlightPlanner.java]
        FlightPlan[FlightPlan.java]
    end

    subgraph "com.flightplanner.algorithms"
        PathOptimizer[PathOptimizer.java]
        AStarNode[AStarNode.java]
    end

    subgraph "com.flightplanner.utils"
        CoordinateConverter[CoordinateConverter.java]
        FlightAnalyzer[FlightAnalyzer.java]
        FlightVisualizer[FlightVisualizer.java]
    end

    subgraph "com.flightplanner.examples"
        MediterraneanRouteExample[MediterraneanRouteExample.java]
    end

    %% Dépendances entre packages

    Main --> examples[com.flightplanner.examples]
    examples --> core[com.flightplanner.core]
    examples --> utils[com.flightplanner.utils]
    core --> models[com.flightplanner.models]
    core --> algorithms[com.flightplanner.algorithms]
    algorithms --> models
    utils --> models
    utils --> core
```

### Responsabilités des packages

| Package | Responsabilité | Classes principales |
|---------|---------------|-------------------|
| `models` | Modèles de données aviation | Waypoint, Airport, VORStation, Runway |
| `core` | Logique métier principale | FlightPlanner, FlightPlan |
| `algorithms` | Algorithmes d'optimisation | PathOptimizer, AStarNode |
| `utils` | Utilitaires et helpers | CoordinateConverter, FlightAnalyzer, FlightVisualizer |
| `examples` | Exemples d'utilisation | MediterraneanRouteExample |

---

## Diagrammes de séquence

### 1. Séquence de planification de route optimisée

```mermaid
sequenceDiagram
    participant Client
    participant FlightPlanner
    participant PathOptimizer
    participant AStarNode
    participant FlightPlan

    Client->>FlightPlanner: planOptimizedRoute(departure, arrival)
    FlightPlanner->>FlightPlanner: updateOptimizer()
    FlightPlanner->>PathOptimizer: findOptimalPath(departure, arrival, true)
    
    loop A* Algorithm
        PathOptimizer->>AStarNode: new AStarNode(waypoint)
        PathOptimizer->>PathOptimizer: getNeighbors(current)
        PathOptimizer->>PathOptimizer: calculateCost(current, neighbor)
        PathOptimizer->>PathOptimizer: heuristic(neighbor, goal)
        AStarNode->>AStarNode: setGScore(tentativeGScore)
        AStarNode->>AStarNode: setFScore(gScore + heuristic)
    end
    
    PathOptimizer->>PathOptimizer: reconstructPath(goalNode)
    PathOptimizer-->>FlightPlanner: optimal path waypoints
    FlightPlanner->>FlightPlan: new FlightPlan(waypoints)
    FlightPlan->>FlightPlan: calculateTotalDistance()
    FlightPlan->>FlightPlan: createSegments()
    FlightPlanner-->>Client: FlightPlan
```

### 2. Séquence de conversion de coordonnées DMS

```mermaid
sequenceDiagram
    participant Client
    participant CoordinateConverter
    participant Pattern
    participant Matcher

    Client->>CoordinateConverter: parseDMSCoordinate("36°45'30\"N 10°15'45\"E")
    CoordinateConverter->>Pattern: matcher(dmsString)
    Pattern-->>CoordinateConverter: Matcher object
    CoordinateConverter->>Matcher: matches()
    Matcher-->>CoordinateConverter: true
    
    loop Extract DMS components
        CoordinateConverter->>Matcher: group(n)
        Matcher-->>CoordinateConverter: component value
    end
    
    CoordinateConverter->>CoordinateConverter: convert to decimal degrees
    CoordinateConverter->>CoordinateConverter: apply direction (N/S/E/W)
    CoordinateConverter-->>Client: double[]{latitude, longitude}
```

### 3. Séquence d'export de plan de vol

```mermaid
sequenceDiagram
    participant Client
    participant FlightPlanner
    participant FlightPlan
    participant FileWriter

    Client->>FlightPlanner: exportFlightPlan(plan, "route.json", "json")
    FlightPlanner->>FlightPlan: exportToJson()
    FlightPlan->>FlightPlan: exportToMap()
    
    loop Convert waypoints to map format
        FlightPlan->>FlightPlan: waypoint data to map
    end
    
    FlightPlan->>FlightPlan: gson.toJson(map)
    FlightPlan-->>FlightPlanner: JSON string
    FlightPlanner->>FileWriter: new FileWriter(filename)
    FlightPlanner->>FileWriter: write(content)
    FlightPlanner->>FileWriter: close()
    FlightPlanner-->>Client: void (success)
```

---

## Diagrammes d'états

### 1. États d'un AStarNode dans l'algorithme A*

```mermaid
stateDiagram-v2
    [*] --> Created : new AStarNode()
    Created --> InOpenSet : added to PriorityQueue
    InOpenSet --> Current : polled from queue
    Current --> InClosedSet : processed
    Current --> Updated : better path found
    Updated --> InOpenSet : re-added to queue
    InClosedSet --> [*] : algorithm complete
    
    state Current {
        [*] --> EvaluatingNeighbors
        EvaluatingNeighbors --> CalculatingCosts
        CalculatingCosts --> UpdatingScores
        UpdatingScores --> CheckingGoal
        CheckingGoal --> GoalReached : if goal
        CheckingGoal --> [*] : if not goal
        GoalReached --> [*]
    }
```

### 2. États d'un FlightPlan

```mermaid
stateDiagram-v2
    [*] --> Initializing : new FlightPlan(waypoints)
    Initializing --> CalculatingDistance : calculateTotalDistance()
    CalculatingDistance --> CreatingSegments : createSegments()
    CreatingSegments --> Ready : construction complete
    
    Ready --> Exporting : export methods called
    Exporting --> ExportingJSON : exportToJson()
    Exporting --> ExportingGPX : exportToGPX()
    Exporting --> ExportingKML : generateKML()
    
    ExportingJSON --> Ready : export complete
    ExportingGPX --> Ready : export complete
    ExportingKML --> Ready : export complete
    
    Ready --> Analyzing : analysis methods called
    Analyzing --> Ready : analysis complete
    
    Ready --> [*] : object destroyed
```

---

## Relations et cardinalités

### 1. Relations entre classes principales

```mermaid
erDiagram
    WAYPOINT {
        string name
        double latitude
        double longitude
        double altitude
        string waypointType
    }
    
    AIRPORT {
        string icaoCode
        string iataCode
        string airportType
        double elevationFt
    }
    
    VORSTATION {
        double frequency
        string identifier
        double magneticVariation
        double rangeNm
        boolean dmeAvailable
    }
    
    RUNWAY {
        string designation
        double lengthFt
        double widthFt
        string surfaceType
    }
    
    FLIGHTPLANNER {
        string id
    }
    
    FLIGHTPLAN {
        double totalDistance
        int waypointCount
    }
    
    SEGMENT {
        double distance
        double bearing
        string legType
    }
    
    %% Relations avec cardinalités
    WAYPOINT ||--o{ AIRPORT : "is-a"
    WAYPOINT ||--o{ VORSTATION : "is-a"
    AIRPORT ||--o{ RUNWAY : "has 0..*"
    FLIGHTPLANNER ||--o{ AIRPORT : "manages 0..*"
    FLIGHTPLANNER ||--o{ VORSTATION : "manages 0..*"
    FLIGHTPLANNER ||--o{ WAYPOINT : "manages 0..*"
    FLIGHTPLANNER ||--o{ FLIGHTPLAN : "creates 0..*"
    FLIGHTPLAN ||--o{ SEGMENT : "contains 1..*"
    FLIGHTPLAN }o--o{ WAYPOINT : "references 2..*"
    SEGMENT ||--|| WAYPOINT : "from waypoint"
    SEGMENT ||--|| WAYPOINT : "to waypoint"
```

### 2. Cardinalités détaillées

| Relation | Cardinalité | Description |
|----------|-------------|-------------|
| Airport → Runway | 1 → 0..* | Un aéroport peut avoir 0 à plusieurs pistes |
| FlightPlanner → Airport | 1 → 0..* | Un planificateur gère 0 à plusieurs aéroports |
| FlightPlanner → VORStation | 1 → 0..* | Un planificateur gère 0 à plusieurs VOR |
| FlightPlanner → Waypoint | 1 → 0..* | Un planificateur gère 0 à plusieurs waypoints |
| FlightPlan → Waypoint | 1 → 2..* | Un plan référence au moins 2 waypoints (départ/arrivée) |
| FlightPlan → Segment | 1 → 1..* | Un plan contient au moins 1 segment |
| Segment → Waypoint (from) | 1 → 1 | Chaque segment a exactement 1 waypoint de départ |
| Segment → Waypoint (to) | 1 → 1 | Chaque segment a exactement 1 waypoint d'arrivée |

### 3. Contraintes d'intégrité

#### Contraintes géographiques
```java
// Coordonnées valides
-90.0 ≤ latitude ≤ 90.0
-180.0 ≤ longitude ≤ 180.0
altitude ≥ -1000.0 (si spécifiée)
```

#### Contraintes aviation
```java
// Fréquences VOR valides
108.0 ≤ frequency ≤ 118.0 (MHz)
rangeNm > 0.0
0.0 ≤ magneticVariation ≤ 360.0

// Pistes d'aéroport
lengthFt > 0.0
widthFt > 0.0
0.0 ≤ headingMagnetic < 360.0
```

#### Contraintes de plan de vol
```java
// Au minimum départ et arrivée
waypoints.size() ≥ 2

// Distance de segment raisonnable
0.0 < segment.distance ≤ MAX_RANGE_NM (500.0)

// Continuité de route
segment[i].to.equals(segment[i+1].from)
```

Cette documentation des diagrammes de classes fournit une vue complète de l'architecture du système Flight Planner Java, facilitant la compréhension, la maintenance et l'évolution du code.
