package roadgraph;

/**
 * Created by ramyashenoy on 12/28/15.
 */
public class MapEdge {
    MapNode start;
    MapNode end;
    String roadName;
    Double length;
    String roadtype;

    public MapNode getStart() {
        return start;
    }

    public MapNode getEnd() {
        return end;
    }

    public String getRoadName() {
        return roadName;
    }

    public Double getLength() {
        return length;
    }

    public String getRoadtype() {
        return roadtype;
    }

    public MapEdge(MapNode from, MapNode to, String roadName, String roadType, double length) {
        this.start = from;
        this.end = to;
        this.roadName = roadName;
        this.length = length;
        this.roadtype = roadType;
    }
}
