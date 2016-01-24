package roadgraph;

import geography.GeographicPoint;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by ramyashenoy on 12/28/15.
 */
public class MapNode {
    private GeographicPoint location;

    private List<MapNode> neighbours;
    private List<MapEdge> adjacentEdges;

    public List<MapEdge> getAdjacentEdges() {
        return adjacentEdges;
    }


    public GeographicPoint getLocation() {

        return location;
    }


    public List<MapNode> getNeighbours() {
        return neighbours;
    }


    public MapNode(GeographicPoint location) {
        this.location = location;
        adjacentEdges = new ArrayList<>();
        neighbours = new ArrayList<>();
    }

    public int getNumEdges(){
        return adjacentEdges.size();
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;

        MapNode mapNode = (MapNode) o;

        if (location != null ? !location.equals(mapNode.location) : mapNode.location != null) return false;
        if (neighbours != null ? !neighbours.equals(mapNode.neighbours) : mapNode.neighbours != null) return false;
        return adjacentEdges != null ? adjacentEdges.equals(mapNode.adjacentEdges) : mapNode.adjacentEdges == null;

    }

}
