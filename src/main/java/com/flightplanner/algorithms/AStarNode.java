package com.flightplanner.algorithms;

import com.flightplanner.models.Waypoint;
import java.util.Objects;

/**
 * Represents a node in the A* pathfinding algorithm.
 * Contains the waypoint and pathfinding-specific data.
 */
public class AStarNode implements Comparable<AStarNode> {
    private final Waypoint waypoint;
    private double gScore; // Cost from start
    private double fScore; // gScore + heuristic
    private AStarNode parent;
    
    /**
     * Create an A* node for a waypoint.
     * 
     * @param waypoint The waypoint this node represents
     */
    public AStarNode(Waypoint waypoint) {
        this.waypoint = waypoint;
        this.gScore = Double.POSITIVE_INFINITY;
        this.fScore = Double.POSITIVE_INFINITY;
        this.parent = null;
    }
    
    /**
     * Create an A* node with initial scores.
     * 
     * @param waypoint The waypoint this node represents
     * @param gScore Initial g-score
     * @param fScore Initial f-score
     */
    public AStarNode(Waypoint waypoint, double gScore, double fScore) {
        this.waypoint = waypoint;
        this.gScore = gScore;
        this.fScore = fScore;
        this.parent = null;
    }
    
    @Override
    public int compareTo(AStarNode other) {
        // Compare by f-score for priority queue
        int result = Double.compare(this.fScore, other.fScore);
        if (result == 0) {
            // If f-scores are equal, compare by h-score (fScore - gScore)
            double thisH = this.fScore - this.gScore;
            double otherH = other.fScore - other.gScore;
            result = Double.compare(thisH, otherH);
        }
        return result;
    }
    
    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        AStarNode aStarNode = (AStarNode) o;
        return Objects.equals(waypoint, aStarNode.waypoint);
    }
    
    @Override
    public int hashCode() {
        return Objects.hash(waypoint);
    }
    
    // Getters and setters
    public Waypoint getWaypoint() { return waypoint; }
    
    public double getGScore() { return gScore; }
    public void setGScore(double gScore) { this.gScore = gScore; }
    
    public double getFScore() { return fScore; }
    public void setFScore(double fScore) { this.fScore = fScore; }
    
    public AStarNode getParent() { return parent; }
    public void setParent(AStarNode parent) { this.parent = parent; }
    
    @Override
    public String toString() {
        return String.format("AStarNode{%s, g=%.1f, f=%.1f}", 
                           waypoint.getName(), gScore, fScore);
    }
}