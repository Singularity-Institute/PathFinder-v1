package com.flightplanner.algorithms;

import com.flightplanner.models.Waypoint;
import com.flightplanner.models.VORStation;
import java.util.*;
import java.util.stream.Collectors;

/**
 * Path optimization using A* algorithm with aviation-specific heuristics.
 * Optimizes flight paths considering VOR preferences and aviation constraints.
 */
public class PathOptimizer {
    private final List<Waypoint> waypoints;
    private final List<VORStation> vorStations;
    private final List<Waypoint> allPoints;
    
    // Optimization parameters
    private static final double MAX_RANGE_NM = 500.0; // Maximum range between waypoints
    private static final double VOR_COST_MULTIPLIER = 0.9; // 10% discount for VOR waypoints
    private static final double LONG_SEGMENT_PENALTY = 1.2; // 20% penalty for segments > 200nm
    private static final double LONG_SEGMENT_THRESHOLD = 200.0; // Threshold for long segment penalty
    
    /**
     * Create a path optimizer with available waypoints and VOR stations.
     * 
     * @param waypoints List of available waypoints
     * @param vorStations List of VOR stations
     */
    public PathOptimizer(List<Waypoint> waypoints, List<VORStation> vorStations) {
        this.waypoints = new ArrayList<>(waypoints);
        this.vorStations = new ArrayList<>(vorStations);
        this.allPoints = new ArrayList<>();
        this.allPoints.addAll(waypoints);
        this.allPoints.addAll(vorStations);
    }
    
    /**
     * Find optimal path between two waypoints using A* algorithm.
     * 
     * @param start Starting waypoint
     * @param goal Destination waypoint
     * @param useVorPreferred Whether to prefer VOR stations in the route
     * @return List of waypoints representing the optimal path
     */
    public List<Waypoint> findOptimalPath(Waypoint start, Waypoint goal, boolean useVorPreferred) {
        return aStar(start, goal, useVorPreferred);
    }
    
    /**
     * A* pathfinding algorithm optimized for aviation.
     * 
     * @param start Starting waypoint
     * @param goal Destination waypoint
     * @param useVorPreferred Whether to prefer VOR stations
     * @return Optimal path as list of waypoints
     */
    private List<Waypoint> aStar(Waypoint start, Waypoint goal, boolean useVorPreferred) {
        // Priority queue for open set (nodes to be evaluated)
        PriorityQueue<AStarNode> openSet = new PriorityQueue<>();
        
        // Set of evaluated nodes
        Set<Waypoint> closedSet = new HashSet<>();
        
        // Maps for tracking best scores
        Map<Waypoint, Double> gScoreMap = new HashMap<>();
        Map<Waypoint, AStarNode> nodeMap = new HashMap<>();
        
        // Initialize start node
        AStarNode startNode = new AStarNode(start, 0.0, heuristic(start, goal));
        openSet.offer(startNode);
        gScoreMap.put(start, 0.0);
        nodeMap.put(start, startNode);
        
        while (!openSet.isEmpty()) {
            AStarNode current = openSet.poll();
            Waypoint currentWaypoint = current.getWaypoint();
            
            // Skip if already processed
            if (closedSet.contains(currentWaypoint)) {
                continue;
            }
            
            // Mark as processed
            closedSet.add(currentWaypoint);
            
            // Check if we've reached the goal
            if (currentWaypoint.equals(goal)) {
                return reconstructPath(current);
            }
            
            // Explore neighbors
            for (Waypoint neighbor : getNeighbors(currentWaypoint)) {
                if (closedSet.contains(neighbor)) {
                    continue;
                }
                
                // Calculate tentative g-score
                double tentativeGScore = current.getGScore() + 
                                       calculateCost(currentWaypoint, neighbor, useVorPreferred);
                
                // Get or create neighbor node
                AStarNode neighborNode = nodeMap.computeIfAbsent(neighbor, AStarNode::new);
                
                // Check if this path is better
                if (tentativeGScore < gScoreMap.getOrDefault(neighbor, Double.POSITIVE_INFINITY)) {
                    // Update neighbor node
                    neighborNode.setParent(current);
                    neighborNode.setGScore(tentativeGScore);
                    neighborNode.setFScore(tentativeGScore + heuristic(neighbor, goal));
                    
                    gScoreMap.put(neighbor, tentativeGScore);
                    
                    // Add to open set if not already there
                    if (!openSet.contains(neighborNode)) {
                        openSet.offer(neighborNode);
                    }
                }
            }
        }
        
        // No path found, return direct path
        return Arrays.asList(start, goal);
    }
    
    /**
     * Get valid neighboring waypoints within reasonable range.
     * 
     * @param waypoint Current waypoint
     * @return List of neighboring waypoints
     */
    private List<Waypoint> getNeighbors(Waypoint waypoint) {
        return allPoints.stream()
                       .filter(point -> !point.equals(waypoint))
                       .filter(point -> waypoint.distanceTo(point) <= MAX_RANGE_NM)
                       .collect(Collectors.toList());
    }
    
    /**
     * Calculate cost between two waypoints with VOR preference.
     * 
     * @param from Source waypoint
     * @param to Destination waypoint
     * @param useVorPreferred Whether to prefer VOR stations
     * @return Cost of the segment
     */
    private double calculateCost(Waypoint from, Waypoint to, boolean useVorPreferred) {
        double baseDistance = from.distanceTo(to);
        double cost = baseDistance;
        
        if (useVorPreferred) {
            // Prefer routes that use VOR stations
            if (to instanceof VORStation) {
                cost *= VOR_COST_MULTIPLIER; // 10% discount for VOR waypoints
            }
            
            // Penalty for very long direct segments
            if (baseDistance > LONG_SEGMENT_THRESHOLD) {
                cost *= LONG_SEGMENT_PENALTY; // 20% penalty for long segments
            }
        }
        
        return cost;
    }
    
    /**
     * Heuristic function for A* (straight-line distance).
     * 
     * @param waypoint Current waypoint
     * @param goal Goal waypoint
     * @return Estimated cost to goal
     */
    private double heuristic(Waypoint waypoint, Waypoint goal) {
        return waypoint.distanceTo(goal);
    }
    
    /**
     * Reconstruct the optimal path from the goal node.
     * 
     * @param goalNode Final node in the path
     * @return List of waypoints representing the path
     */
    private List<Waypoint> reconstructPath(AStarNode goalNode) {
        List<Waypoint> path = new ArrayList<>();
        AStarNode current = goalNode;
        
        while (current != null) {
            path.add(current.getWaypoint());
            current = current.getParent();
        }
        
        Collections.reverse(path);
        return path;
    }
    
    /**
     * Optimize waypoint sequence using nearest neighbor heuristic for TSP-like problems.
     * 
     * @param waypoints List of waypoints to optimize
     * @return Optimized sequence of waypoints
     */
    public List<Waypoint> optimizeWaypointSequence(List<Waypoint> waypoints) {
        if (waypoints.size() <= 2) {
            return new ArrayList<>(waypoints);
        }
        
        if (waypoints.size() <= 10) {
            return nearestNeighborTSP(waypoints);
        } else {
            // For larger sets, use the same heuristic (could be enhanced with genetic algorithm)
            return nearestNeighborTSP(waypoints);
        }
    }
    
    /**
     * Solve TSP using nearest neighbor heuristic.
     * 
     * @param waypoints Waypoints to visit
     * @return Optimized order of waypoints
     */
    private List<Waypoint> nearestNeighborTSP(List<Waypoint> waypoints) {
        if (waypoints.isEmpty()) {
            return new ArrayList<>();
        }
        
        List<Waypoint> unvisited = new ArrayList<>(waypoints.subList(1, waypoints.size()));
        List<Waypoint> path = new ArrayList<>();
        path.add(waypoints.get(0));
        
        Waypoint current = waypoints.get(0);
        
        while (!unvisited.isEmpty()) {
            final Waypoint currentFinal = current;  // Create final reference for lambda
            Waypoint nearest = unvisited.stream()
                                       .min((w1, w2) -> Double.compare(currentFinal.distanceTo(w1), currentFinal.distanceTo(w2)))
                                       .orElse(null);
            
            if (nearest != null) {
                path.add(nearest);
                unvisited.remove(nearest);
                current = nearest;
            }
        }
        
        return path;
    }
    
    /**
     * Find VOR-optimized route with maximum VOR coverage.
     * 
     * @param start Starting waypoint
     * @param goal Destination waypoint
     * @return Tuple of (optimal path, covering VOR stations)
     */
    public PathWithVORCoverage findVORRoute(Waypoint start, Waypoint goal) {
        // Find path using VOR preference
        List<Waypoint> path = findOptimalPath(start, goal, true);
        
        // Identify which VOR stations provide coverage
        Set<VORStation> coverageVORs = new HashSet<>();
        for (VORStation vor : vorStations) {
            for (Waypoint waypoint : path) {
                if (vor.isInRange(waypoint) && !path.contains(vor)) {
                    coverageVORs.add(vor);
                    break;
                }
            }
        }
        
        return new PathWithVORCoverage(path, new ArrayList<>(coverageVORs));
    }
    
    /**
     * Result class for VOR route optimization.
     */
    public static class PathWithVORCoverage {
        private final List<Waypoint> path;
        private final List<VORStation> coverageVORs;
        
        public PathWithVORCoverage(List<Waypoint> path, List<VORStation> coverageVORs) {
            this.path = path;
            this.coverageVORs = coverageVORs;
        }
        
        public List<Waypoint> getPath() { return path; }
        public List<VORStation> getCoverageVORs() { return coverageVORs; }
    }
}