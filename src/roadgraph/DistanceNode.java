package roadgraph;

/**
 * Created by ramyashenoy on 1/5/16.
 */
public class DistanceNode implements Comparable {
    MapNode mapNode;
    Double distance;
    Double heuristicDistance;

    public DistanceNode(MapNode mapNode, Double distance) {
        this.mapNode = mapNode;
        this.distance = distance;
        heuristicDistance = 0.0;
    }

    public DistanceNode(MapNode mapNode, Double distance, Double heuristicDistance) {
        this.mapNode = mapNode;
        this.distance = distance;
        this.heuristicDistance = heuristicDistance;
    }

    public MapNode getMapNode() {
        return mapNode;
    }

    public Double getDistance() {
        return distance + heuristicDistance;
    }


    @Override
    public int compareTo(Object o) {
        DistanceNode distanceNode= (DistanceNode) o;
        if(this.getDistance() > distanceNode.getDistance()){
            return 1;
        }
        return -1;
    }
}
