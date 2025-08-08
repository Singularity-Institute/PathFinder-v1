# Analyse des Contraintes Aviation - Flight Planner Java

## Table des Matières

1. [Contraintes réglementaires](#contraintes-réglementaires)
2. [Contraintes techniques aviation](#contraintes-techniques-aviation)
3. [Contraintes de sécurité](#contraintes-de-sécurité)
4. [Contraintes opérationnelles](#contraintes-opérationnelles)
5. [Contraintes algorithmiques](#contraintes-algorithmiques)
6. [Modélisation mathématique](#modélisation-mathématique)
7. [Implémentation des contraintes](#implémentation-des-contraintes)
8. [Validation et tests](#validation-et-tests)

---

## Contraintes réglementaires

### 1. Standards OACI (Organisation de l'Aviation Civile Internationale)

#### Annexe 2 : Règles de l'air
```java
public enum FlightRules {
    VFR,  // Vol à vue (Visual Flight Rules)
    IFR   // Vol aux instruments (Instrument Flight Rules)
}

public class ICAOCompliance {
    // Règle OACI : Séparation minimale entre aéronefs
    public static final double MIN_LATERAL_SEPARATION_NM = 5.0;
    public static final double MIN_VERTICAL_SEPARATION_FT = 1000.0;
    
    // Règle OACI : Niveaux de vol
    public static boolean isValidFlightLevel(int flightLevel, double magneticTrack) {
        // Niveaux pairs pour routes 000-179°, impairs pour 180-359°
        boolean isEastbound = magneticTrack >= 0 && magneticTrack < 180;
        return (flightLevel % 2 == 0) != isEastbound; // XOR logic
    }
}
```

#### Annexe 11 : Services de la circulation aérienne
```java
public class ATSRouteConstraints {
    // Routes ATS (Air Traffic Services)
    private Map<String, ATSRoute> publishedRoutes;
    
    public boolean isRoutePublished(List<Waypoint> route) {
        // Vérifier si la route utilise des segments ATS publiés
        for (int i = 0; i < route.size() - 1; i++) {
            if (!hasPublishedSegment(route.get(i), route.get(i + 1))) {
                return false;
            }
        }
        return true;
    }
    
    // MEA (Minimum En-route Altitude)
    public int getMinimumEnrouteAltitude(Waypoint from, Waypoint to) {
        ATSSegment segment = findATSSegment(from, to);
        return segment != null ? segment.getMEA() : getGridMEA(from, to);
    }
}
```

### 2. Réglementation européenne (EASA)

#### Règlement (UE) 2018/1139 : Sécurité aérienne
```java
public class EASACompliance {
    
    // Performance Based Navigation (PBN)
    public enum NavigationSpecification {
        RNAV_1(1.0),    // RNP 1.0 nautical miles
        RNAV_5(5.0),    // RNP 5.0 nautical miles
        RNP_0_3(0.3),   // RNP 0.3 nautical miles
        RNP_APCH(0.1);  // RNP Approach 0.1 nautical miles
        
        private final double accuracy;
        NavigationSpecification(double accuracy) { this.accuracy = accuracy; }
    }
    
    // Vérification de la précision de navigation requise
    public boolean meetsNavigationRequirement(FlightPlan plan, 
                                             NavigationSpecification required) {
        double maxDeviation = calculateMaxDeviation(plan);
        return maxDeviation <= required.accuracy;
    }
}
```

### 3. Réglementation française (DGAC)

#### AIP France : Espaces aériens contrôlés
```java
public class FrenchAirspaceConstraints {
    
    // Zones à statut particulier
    public enum AirspaceType {
        TMA,        // Terminal Control Area
        CTR,        // Control Zone  
        AWY,        // Airway
        R_ZONE,     // Restricted Area
        D_ZONE,     // Dangerous Area
        P_ZONE      // Prohibited Area
    }
    
    public class AirspaceRestriction {
        private Polygon boundary;
        private int floorAltitude;
        private int ceilingAltitude;
        private TimeWindow activeHours;
        private Set<String> applicableAircraft;
        
        public boolean isActiveAt(LocalDateTime time) {
            return activeHours.contains(time);
        }
        
        public boolean appliesToFlight(FlightPlan plan, AircraftType aircraft) {
            return applicableAircraft.isEmpty() || 
                   applicableAircraft.contains(aircraft.getCategory());
        }
    }
}
```

---

## Contraintes techniques aviation

### 1. Limitations de performance aéronef

#### Enveloppe de vol
```java
public class AircraftPerformanceConstraints {
    
    public class PerformanceEnvelope {
        private int serviceCeiling;        // Plafond pratique (ft)
        private double maxRange;           // Autonomie maximale (nm)
        private double cruiseSpeed;        // Vitesse de croisière (kts)
        private double maxTakeoffWeight;   // Masse maximale au décollage (kg)
        private double fuelCapacity;       // Capacité carburant (liters)
        
        public boolean canExecuteSegment(Waypoint from, Waypoint to, int altitude) {
            double distance = from.distanceTo(to);
            double requiredFuel = calculateFuelForSegment(from, to, altitude);
            
            return distance <= maxRange && 
                   altitude <= serviceCeiling &&
                   requiredFuel <= fuelCapacity;
        }
    }
    
    // Calcul de la consommation carburant
    public double calculateFuelConsumption(FlightPlan plan, AircraftType aircraft) {
        double totalFuel = 0.0;
        
        for (FlightPlan.Segment segment : plan.getSegments()) {
            // Consommation de base + corrections météo
            double segmentFuel = aircraft.getFuelFlowRate() * 
                               calculateSegmentTime(segment, aircraft);
            
            // Corrections altitude
            segmentFuel *= getAltitudeCorrection(segment.getAltitude());
            
            // Corrections météo (vent de face/arrière)
            segmentFuel *= getWeatherCorrection(segment);
            
            totalFuel += segmentFuel;
        }
        
        // Ajouter réserves obligatoires
        totalFuel += calculateRequiredReserves(plan, aircraft);
        
        return totalFuel;
    }
}
```

#### Limitations de piste
```java
public class RunwayConstraints {
    
    public class RunwayPerformanceCalculator {
        
        // Distance de décollage requise
        public double calculateTakeoffDistance(Airport airport, AircraftType aircraft, 
                                             WeatherConditions weather) {
            Runway longestRunway = airport.getLongestRunway().orElseThrow();
            
            double baseDistance = aircraft.getBaseTakeoffDistance();
            
            // Corrections environnementales
            baseDistance *= getElevationCorrection(airport.getElevationFt());
            baseDistance *= getTemperatureCorrection(weather.getTemperature());
            baseDistance *= getWindCorrection(weather.getWind(), longestRunway.getHeading());
            baseDistance *= getSurfaceCorrection(longestRunway.getSurfaceType());
            
            return baseDistance;
        }
        
        // Vérification capacité d'accueil
        public boolean canAccommodateAircraft(Airport airport, AircraftType aircraft,
                                            WeatherConditions weather) {
            double requiredDistance = calculateTakeoffDistance(airport, aircraft, weather);
            double availableDistance = airport.getLongestRunway()
                                            .map(Runway::getLengthFt)
                                            .orElse(0.0);
            
            return availableDistance >= requiredDistance * SAFETY_MARGIN; // 1.15
        }
    }
}
```

### 2. Contraintes de navigation

#### Précision des systèmes de navigation
```java
public class NavigationAccuracyConstraints {
    
    // Erreurs de navigation selon le système utilisé
    public enum NavigationSystem {
        GPS(3.0),           // GPS civil : ±3m (95%)
        WAAS(1.0),          // Wide Area Augmentation System : ±1m
        VOR(5.0),           // VHF Omnidirectional Range : ±5°
        NDB(10.0),          // Non-Directional Beacon : ±10°
        ILS(0.1),           // Instrument Landing System : ±0.1°
        DME(0.5);           // Distance Measuring Equipment : ±0.5nm
        
        private final double accuracy; // Précision typique
        
        NavigationSystem(double accuracy) {
            this.accuracy = accuracy;
        }
    }
    
    // Calcul de l'erreur de navigation cumulée
    public double calculateNavigationError(FlightPlan plan) {
        double totalError = 0.0;
        
        for (FlightPlan.Segment segment : plan.getSegments()) {
            NavigationSystem primaryNav = determinePrimaryNavigation(segment);
            double segmentError = primaryNav.accuracy * segment.getDistance();
            
            // Erreur cumulée (propagation d'incertitude)
            totalError = Math.sqrt(totalError * totalError + segmentError * segmentError);
        }
        
        return totalError;
    }
    
    // Déterminer le système de navigation optimal
    private NavigationSystem determinePrimaryNavigation(FlightPlan.Segment segment) {
        // Priorité : GPS/WAAS > VOR > NDB
        if (hasGPSCoverage(segment)) {
            return hasWAASCoverage(segment) ? NavigationSystem.WAAS : NavigationSystem.GPS;
        } else if (hasVORCoverage(segment)) {
            return NavigationSystem.VOR;
        } else {
            return NavigationSystem.NDB; // Fallback
        }
    }
}
```

### 3. Contraintes de communication

#### Couverture radio VHF
```java
public class CommunicationConstraints {
    
    // Portée radio VHF (line-of-sight)
    public double calculateVHFRange(int altitude1, int altitude2) {
        // Formule radar horizon
        double range1 = 1.23 * Math.sqrt(altitude1); // Portée station 1
        double range2 = 1.23 * Math.sqrt(altitude2); // Portée station 2
        return range1 + range2; // Portée totale
    }
    
    // Zones de silence radio
    public List<RadioShadowZone> identifyRadioShadows(FlightPlan plan) {
        List<RadioShadowZone> shadows = new ArrayList<>();
        
        for (FlightPlan.Segment segment : plan.getSegments()) {
            // Vérifier couverture ATC le long du segment
            if (!hasATCCoverage(segment)) {
                shadows.add(new RadioShadowZone(
                    segment.getFrom(),
                    segment.getTo(),
                    calculateShadowDuration(segment)
                ));
            }
        }
        
        return shadows;
    }
}
```

---

## Contraintes de sécurité

### 1. Séparation entre aéronefs

#### Standards ICAO de séparation
```java
public class SeparationConstraints {
    
    // Séparation latérale minimale
    public static final double MIN_LATERAL_SEPARATION_NM = 5.0;
    
    // Séparation verticale selon l'altitude
    public static double getMinVerticalSeparation(int altitude) {
        if (altitude <= 29000) {
            return 1000.0; // 1000 ft sous FL290
        } else if (altitude <= 41000) {
            return 2000.0; // 2000 ft entre FL290-FL410
        } else {
            return 4000.0; // 4000 ft au-dessus FL410
        }
    }
    
    // Détection de conflits potentiels
    public class ConflictDetector {
        
        public List<TrafficConflict> detectConflicts(FlightPlan plan, 
                                                   List<OtherAircraft> traffic,
                                                   LocalDateTime departureTime) {
            List<TrafficConflict> conflicts = new ArrayList<>();
            
            for (OtherAircraft other : traffic) {
                ConflictPoint conflict = calculateClosestApproach(plan, other, departureTime);
                
                if (conflict.getHorizontalSeparation() < MIN_LATERAL_SEPARATION_NM ||
                    conflict.getVerticalSeparation() < getMinVerticalSeparation(conflict.getAltitude())) {
                    conflicts.add(new TrafficConflict(other, conflict));
                }
            }
            
            return conflicts;
        }
    }
}
```

### 2. Évitement des obstacles

#### Contraintes de relief (MEF - Maximum Elevation Figure)
```java
public class TerrainConstraints {
    
    // Altitude minimale de sécurité
    public int getMinimumSafeAltitude(Waypoint center, double radiusNM) {
        // Chercher le point le plus élevé dans le rayon
        int maxElevation = getMaxElevationInRadius(center, radiusNM);
        
        // Ajouter marge de sécurité (1000 ft minimum)
        return maxElevation + Math.max(1000, (int)(maxElevation * 0.1));
    }
    
    // Évitement du relief
    public class TerrainAvoidance {
        
        public boolean isSafeAltitude(FlightPlan.Segment segment, int altitude) {
            // Vérifier tout le long du segment
            int samples = Math.max(10, (int)(segment.getDistance() / 10.0)); // Échantillon tous les 10nm
            
            for (int i = 0; i <= samples; i++) {
                double ratio = (double) i / samples;
                Waypoint samplePoint = interpolatePosition(segment, ratio);
                int msa = getMinimumSafeAltitude(samplePoint, 5.0); // Rayon 5nm
                
                if (altitude < msa) {
                    return false;
                }
            }
            
            return true;
        }
    }
}
```

### 3. Conditions météorologiques

#### Critères de sécurité météo
```java
public class WeatherSafetyConstraints {
    
    public enum WeatherHazard {
        THUNDERSTORM(50.0),      // Évitement 50nm
        SEVERE_TURBULENCE(20.0), // Évitement 20nm
        ICING(10.0),             // Évitement 10nm
        LOW_VISIBILITY(5.0),     // Évitement 5nm
        WIND_SHEAR(15.0);        // Évitement 15nm
        
        private final double avoidanceRadius;
        WeatherHazard(double radius) { this.avoidanceRadius = radius; }
    }
    
    public class WeatherAvoidanceCalculator {
        
        public boolean isRouteSafe(FlightPlan plan, WeatherConditions weather) {
            for (FlightPlan.Segment segment : plan.getSegments()) {
                List<WeatherHazard> hazards = identifyWeatherHazards(segment, weather);
                
                for (WeatherHazard hazard : hazards) {
                    if (!hasAdequateAvoidance(segment, hazard)) {
                        return false;
                    }
                }
            }
            
            return true;
        }
        
        // Modifier route pour éviter météo dangereuse
        public FlightPlan avoidWeatherHazards(FlightPlan originalPlan, 
                                            WeatherConditions weather) {
            List<Waypoint> safeRoute = new ArrayList<>();
            safeRoute.add(originalPlan.getWaypoints().get(0)); // Départ
            
            for (int i = 1; i < originalPlan.getWaypoints().size(); i++) {
                Waypoint target = originalPlan.getWaypoints().get(i);
                Waypoint current = safeRoute.get(safeRoute.size() - 1);
                
                // Vérifier si le segment direct est sûr
                if (isSegmentSafe(current, target, weather)) {
                    safeRoute.add(target);
                } else {
                    // Chercher waypoints de contournement
                    List<Waypoint> detour = findWeatherAvoidanceRoute(current, target, weather);
                    safeRoute.addAll(detour);
                }
            }
            
            return new FlightPlan(safeRoute);
        }
    }
}
```

---

## Contraintes opérationnelles

### 1. Gestion du trafic aérien

#### Slots et créneaux horaires
```java
public class ATCConstraints {
    
    public class SlotAllocation {
        private LocalDateTime allocatedTime;
        private Duration tolerance;        // Tolérance ±5 minutes
        private String slotReference;
        
        public boolean isValidDepartureTime(LocalDateTime proposedTime) {
            return Math.abs(Duration.between(allocatedTime, proposedTime).toMinutes()) 
                   <= tolerance.toMinutes();
        }
    }
    
    // Contraintes de flux de trafic
    public class TrafficFlow {
        private String routeDesignation;
        private int maxHourlyCapacity;
        private Map<LocalTime, Integer> currentLoad;
        
        public boolean canAcceptAdditionalFlight(LocalDateTime time) {
            LocalTime hour = time.toLocalTime().truncatedTo(ChronoUnit.HOURS);
            int currentFlights = currentLoad.getOrDefault(hour, 0);
            return currentFlights < maxHourlyCapacity;
        }
    }
}
```

### 2. Coûts opérationnels

#### Redevances de survol
```java
public class OverflightCostConstraints {
    
    // Coûts par pays/région
    private Map<String, Double> countryRates; // EUR per 100kg per 100nm
    
    public double calculateOverflightCosts(FlightPlan plan, AircraftType aircraft) {
        double totalCost = 0.0;
        
        for (FlightPlan.Segment segment : plan.getSegments()) {
            List<String> countriesOverflown = identifyCountries(segment);
            
            for (String country : countriesOverflown) {
                double rate = countryRates.getOrDefault(country, 0.0);
                double segmentCost = (aircraft.getMaxTakeoffWeight() / 100.0) * 
                                   (segment.getDistance() / 100.0) * rate;
                totalCost += segmentCost;
            }
        }
        
        return totalCost;
    }
    
    // Optimisation basée sur les coûts
    public FlightPlan optimizeForCosts(Airport departure, Airport arrival, 
                                     AircraftType aircraft) {
        return pathOptimizer.findOptimalPath(
            departure, arrival,
            (from, to) -> calculateSegmentCost(from, to, aircraft) // Fonction de coût personnalisée
        );
    }
}
```

### 3. Contraintes carburant

#### Règles de carburant ICAO
```java
public class FuelConstraints {
    
    // Carburant minimum requis selon ICAO Annex 6
    public class FuelRequirements {
        
        public double calculateMinimumFuel(FlightPlan plan, AircraftType aircraft) {
            // 1. Carburant pour le vol (taxi + vol + approche)
            double tripFuel = calculateTripFuel(plan, aircraft);
            
            // 2. Carburant pour déroutement vers alternance
            double alternateFuel = calculateAlternateFuel(plan, aircraft);
            
            // 3. Carburant de réserve finale (30 minutes de vol)
            double finalReserve = aircraft.getFuelFlowRate() * 0.5; // 30 min
            
            // 4. Carburant de réserve contingence (5% du trip fuel)
            double contingencyReserve = tripFuel * 0.05;
            
            return tripFuel + alternateFuel + finalReserve + contingencyReserve;
        }
        
        // Vérification de faisabilité
        public boolean isFuelSufficient(FlightPlan plan, AircraftType aircraft, 
                                      double availableFuel) {
            double requiredFuel = calculateMinimumFuel(plan, aircraft);
            return availableFuel >= requiredFuel * SAFETY_MARGIN; // 1.1
        }
    }
}
```

---

## Contraintes algorithmiques

### 1. Contraintes de performance computationnelle

#### Limites de temps de calcul
```java
public class PerformanceConstraints {
    
    // Contraintes temps réel
    public static final long MAX_COMPUTATION_TIME_MS = 5000; // 5 secondes max
    public static final int MAX_WAYPOINTS_FULL_OPTIMIZATION = 20;
    public static final int MAX_ITERATIONS_A_STAR = 10000;
    
    public class TimeoutOptimizer extends PathOptimizer {
        
        @Override
        public List<Waypoint> findOptimalPath(Waypoint start, Waypoint goal, 
                                            boolean useVorPreferred) {
            long startTime = System.currentTimeMillis();
            
            try {
                return super.findOptimalPath(start, goal, useVorPreferred);
            } catch (Exception e) {
                long elapsed = System.currentTimeMillis() - startTime;
                
                if (elapsed > MAX_COMPUTATION_TIME_MS) {
                    logger.warn("Optimization timeout, using fallback algorithm");
                    return fallbackOptimizer.findPath(start, goal);
                } else {
                    throw e;
                }
            }
        }
    }
}
```

### 2. Contraintes mémoire

#### Gestion des ressources
```java
public class MemoryConstraints {
    
    // Limites mémoire pour éviter OutOfMemoryError
    public static final int MAX_OPEN_NODES = 50000;
    public static final int MAX_CACHED_DISTANCES = 100000;
    
    public class MemoryAwareOptimizer {
        private final LRUCache<String, Double> distanceCache;
        
        public MemoryAwareOptimizer() {
            this.distanceCache = new LRUCache<>(MAX_CACHED_DISTANCES);
        }
        
        // Nettoyage périodique de la mémoire
        public void periodicCleanup() {
            if (openNodes.size() > MAX_OPEN_NODES * 0.8) {
                // Nettoyer les nœuds les moins prometteurs
                cleanupLeastPromisingNodes();
            }
            
            // Forcer garbage collection si nécessaire
            long freeMemory = Runtime.getRuntime().freeMemory();
            long totalMemory = Runtime.getRuntime().totalMemory();
            
            if (freeMemory / (double) totalMemory < 0.1) { // <10% mémoire libre
                System.gc();
            }
        }
    }
}
```

---

## Modélisation mathématique

### 1. Fonction objective multi-critères

#### Formulation mathématique du problème d'optimisation
```
Minimiser : f(x) = w₁·d(x) + w₂·t(x) + w₃·c(x) + w₄·r(x)

Où :
- x = route candidate (séquence de waypoints)
- d(x) = distance totale
- t(x) = temps de vol total  
- c(x) = coût total (carburant + redevances)
- r(x) = facteur de risque (météo + trafic)
- wᵢ = poids des critères (∑wᵢ = 1)

Contraintes :
- Continuité : wᵢ₊₁.position = wᵢ.position + segment_length
- Distance max : ||wᵢ₊₁ - wᵢ|| ≤ MAX_SEGMENT_LENGTH
- Altitude min : altitude(wᵢ) ≥ MSA(wᵢ)
- Carburant : fuel_consumed(x) ≤ fuel_available
- Temps : flight_time(x) ≤ max_flight_time
```

### 2. Modèle probabiliste des incertitudes

#### Propagation d'erreurs de navigation
```java
public class UncertaintyModel {
    
    // Modèle d'erreur de navigation (distribution normale)
    public class NavigationError {
        private double meanError;     // Erreur moyenne (nm)
        private double stdDeviation;  // Écart-type (nm)
        
        // Propagation d'incertitude le long de la route
        public NavigationError propagateError(List<FlightPlan.Segment> segments) {
            double totalVariance = 0.0;
            
            for (FlightPlan.Segment segment : segments) {
                // Erreur proportionnelle à la distance
                double segmentVariance = Math.pow(stdDeviation * segment.getDistance(), 2);
                totalVariance += segmentVariance;
            }
            
            return new NavigationError(meanError, Math.sqrt(totalVariance));
        }
    }
    
    // Probabilité d'arrivée dans un rayon donné
    public double calculateArrivalProbability(FlightPlan plan, double radiusNm) {
        NavigationError finalError = calculateFinalError(plan);
        
        // Probabilité que l'erreur finale soit dans le rayon
        // P(|error| ≤ radius) = 2 * Φ(radius/σ) - 1
        double standardizedRadius = radiusNm / finalError.stdDeviation;
        return 2 * normalCDF(standardizedRadius) - 1;
    }
}
```

---

## Implémentation des contraintes

### 1. Architecture de validation

```java
public class ConstraintValidator {
    
    private List<FlightPlanConstraint> constraints;
    
    public interface FlightPlanConstraint {
        ValidationResult validate(FlightPlan plan, FlightContext context);
        String getConstraintName();
        ConstraintPriority getPriority();
    }
    
    public enum ConstraintPriority {
        CRITICAL,    // Violation = route impraticable
        HIGH,        // Violation = route dangereuse
        MEDIUM,      // Violation = route non optimale
        LOW          // Violation = simple avertissement
    }
    
    public ValidationResult validatePlan(FlightPlan plan, FlightContext context) {
        ValidationResult result = new ValidationResult();
        
        for (FlightPlanConstraint constraint : constraints) {
            ValidationResult constraintResult = constraint.validate(plan, context);
            result.merge(constraintResult);
            
            // Arrêter si contrainte critique violée
            if (constraintResult.hasCriticalViolations()) {
                result.setFatalError(constraint.getConstraintName());
                break;
            }
        }
        
        return result;
    }
}
```

### 2. Contraintes spécifiques implémentées

#### Contrainte de distance maximale
```java
public class MaxDistanceConstraint implements FlightPlanConstraint {
    
    private static final double MAX_SEGMENT_DISTANCE = 500.0; // nm
    
    @Override
    public ValidationResult validate(FlightPlan plan, FlightContext context) {
        ValidationResult result = new ValidationResult();
        
        for (FlightPlan.Segment segment : plan.getSegments()) {
            if (segment.getDistance() > MAX_SEGMENT_DISTANCE) {
                result.addViolation(
                    ConstraintPriority.CRITICAL,
                    String.format("Segment %s → %s exceeds maximum distance: %.1f nm > %.1f nm",
                        segment.getFrom().getName(),
                        segment.getTo().getName(),
                        segment.getDistance(),
                        MAX_SEGMENT_DISTANCE)
                );
            }
        }
        
        return result;
    }
    
    @Override
    public String getConstraintName() { return "MaxSegmentDistance"; }
    
    @Override
    public ConstraintPriority getPriority() { return ConstraintPriority.CRITICAL; }
}
```

#### Contrainte de couverture VOR
```java
public class VORCoverageConstraint implements FlightPlanConstraint {
    
    private static final double MIN_VOR_COVERAGE = 0.7; // 70%
    
    @Override
    public ValidationResult validate(FlightPlan plan, FlightContext context) {
        ValidationResult result = new ValidationResult();
        
        double totalDistance = plan.getTotalDistance();
        double coveredDistance = calculateVORCoveredDistance(plan, context.getVorStations());
        double coverageRatio = coveredDistance / totalDistance;
        
        if (coverageRatio < MIN_VOR_COVERAGE) {
            result.addViolation(
                ConstraintPriority.HIGH,
                String.format("Insufficient VOR coverage: %.1f%% < %.1f%%",
                    coverageRatio * 100, MIN_VOR_COVERAGE * 100)
            );
        }
        
        return result;
    }
    
    private double calculateVORCoveredDistance(FlightPlan plan, List<VORStation> vorStations) {
        double coveredDistance = 0.0;
        
        for (FlightPlan.Segment segment : plan.getSegments()) {
            boolean segmentCovered = vorStations.stream()
                .anyMatch(vor -> isSegmentCoveredByVOR(segment, vor));
            
            if (segmentCovered) {
                coveredDistance += segment.getDistance();
            }
        }
        
        return coveredDistance;
    }
}
```

---

## Validation et tests

### 1. Tests de contraintes

```java
@TestMethodOrder(OrderAnnotation.class)
public class ConstraintValidationTest {
    
    private FlightPlanner planner;
    private ConstraintValidator validator;
    private Airport tunis, santorini;
    
    @BeforeEach
    void setUp() {
        planner = new FlightPlanner();
        validator = new ConstraintValidator();
        
        // Charger données de test
        MediterraneanData data = CoordinateConverter.generateMediterraneanData();
        data.getAirports().forEach(planner::addAirport);
        data.getVors().forEach(planner::addVORStation);
        
        tunis = data.getAirports().stream()
            .filter(a -> "DTTA".equals(a.getIcaoCode()))
            .findFirst().orElseThrow();
            
        santorini = data.getAirports().stream()
            .filter(a -> "LGSR".equals(a.getIcaoCode()))
            .findFirst().orElseThrow();
    }
    
    @Test
    @Order(1)
    @DisplayName("Route directe respecte contraintes de base")
    void testDirectRouteConstraints() {
        FlightPlan directPlan = planner.planDirectRoute(tunis, santorini);
        FlightContext context = new FlightContext(planner.getVORStations());
        
        ValidationResult result = validator.validatePlan(directPlan, context);
        
        assertFalse(result.hasCriticalViolations(), "Route directe ne doit pas violer contraintes critiques");
        assertTrue(result.isValid(), "Route directe doit être valide");
        
        // Vérifier contrainte de distance
        assertTrue(directPlan.getTotalDistance() < 1000.0, 
            "Distance totale doit être raisonnable");
    }
    
    @Test
    @Order(2)
    @DisplayName("Route optimisée améliore couverture VOR")
    void testOptimizedRouteVORCoverage() {
        FlightPlan directPlan = planner.planDirectRoute(tunis, santorini);
        FlightPlan optimizedPlan = planner.planOptimizedRoute(tunis, santorini);
        
        FlightContext context = new FlightContext(planner.getVORStations());
        
        double directCoverage = calculateVORCoverage(directPlan, context.getVorStations());
        double optimizedCoverage = calculateVORCoverage(optimizedPlan, context.getVorStations());
        
        assertTrue(optimizedCoverage >= directCoverage, 
            "Route optimisée doit avoir couverture VOR >= route directe");
        
        if (optimizedPlan.getWaypoints().size() > 2) {
            assertTrue(optimizedCoverage > 0.5, 
                "Route optimisée doit avoir couverture VOR significative");
        }
    }
    
    @Test
    @Order(3)
    @DisplayName("Contraintes de segment respectées")
    void testSegmentConstraints() {
        FlightPlan plan = planner.planOptimizedRoute(tunis, santorini);
        
        for (FlightPlan.Segment segment : plan.getSegments()) {
            assertTrue(segment.getDistance() > 0, 
                "Distance de segment doit être positive");
            assertTrue(segment.getDistance() <= 500.0, 
                "Distance de segment ne doit pas dépasser 500nm");
            assertTrue(segment.getBearing() >= 0 && segment.getBearing() < 360, 
                "Cap doit être valide [0-360°)");
        }
    }
    
    @Test
    @Order(4)
    @DisplayName("Test de performance sous contraintes")
    void testPerformanceConstraints() {
        long startTime = System.currentTimeMillis();
        
        FlightPlan plan = planner.planOptimizedRoute(tunis, santorini);
        
        long duration = System.currentTimeMillis() - startTime;
        
        assertTrue(duration < 5000, 
            "Calcul doit se terminer en moins de 5 secondes");
        assertNotNull(plan, "Plan ne doit pas être null");
        assertTrue(plan.getWaypoints().size() >= 2, 
            "Plan doit avoir au moins départ et arrivée");
    }
    
    // Méthode utilitaire
    private double calculateVORCoverage(FlightPlan plan, List<VORStation> vors) {
        double totalDistance = plan.getTotalDistance();
        double coveredDistance = 0.0;
        
        for (FlightPlan.Segment segment : plan.getSegments()) {
            boolean covered = vors.stream()
                .anyMatch(vor -> vor.isInRange(segment.getFrom()) || 
                               vor.isInRange(segment.getTo()));
            if (covered) {
                coveredDistance += segment.getDistance();
            }
        }
        
        return totalDistance > 0 ? coveredDistance / totalDistance : 0.0;
    }
}
```

### 2. Tests de régression

```java
@Test
@DisplayName("Test de régression - Routes connues")
void testKnownRoutes() {
    // Routes de référence avec résultats attendus
    Map<String, ExpectedResult> referenceRoutes = Map.of(
        "DTTA-LGSR-direct", new ExpectedResult(734.7, 2, 0.2),
        "DTTA-LGSR-optimized", new ExpectedResult(750.9, 6, 0.85),
        "DTTA-LGSR-vor", new ExpectedResult(763.2, 8, 0.95)
    );
    
    for (Map.Entry<String, ExpectedResult> entry : referenceRoutes.entrySet()) {
        String routeId = entry.getKey();
        ExpectedResult expected = entry.getValue();
        
        FlightPlan actualPlan = calculateRoute(routeId);
        
        // Vérifier distance (tolérance ±1%)
        assertEquals(expected.distance, actualPlan.getTotalDistance(), 
            expected.distance * 0.01, 
            "Distance pour " + routeId);
        
        // Vérifier nombre de waypoints
        assertEquals(expected.waypointCount, actualPlan.getWaypoints().size(),
            "Nombre waypoints pour " + routeId);
        
        // Vérifier couverture VOR (tolérance ±5%)
        double actualCoverage = calculateVORCoverage(actualPlan, planner.getVORStations());
        assertEquals(expected.vorCoverage, actualCoverage, 0.05,
            "Couverture VOR pour " + routeId);
    }
}

private static class ExpectedResult {
    final double distance;
    final int waypointCount;
    final double vorCoverage;
    
    ExpectedResult(double distance, int waypointCount, double vorCoverage) {
        this.distance = distance;
        this.waypointCount = waypointCount;
        this.vorCoverage = vorCoverage;
    }
}
```

Cette analyse complète des contraintes aviation montre comment le système Flight Planner Java intègre les exigences réglementaires, techniques et opérationnelles dans ses algorithmes d'optimisation, tout en maintenant la sécurité et l'efficacité des routes calculées.