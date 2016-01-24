/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import geography.GeographicPoint;
import util.GraphLoader;

import java.util.*;
import java.util.function.Consumer;

/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph {

	private HashMap<GeographicPoint, MapNode> graph;

	/**
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		graph = new HashMap<>();
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		return graph.size();
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{

		return graph.keySet();
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		Iterator it = graph.entrySet().iterator();
		int numEdges = 0;
		while (it.hasNext()) {
			Map.Entry pair = (Map.Entry)it.next();
			MapNode value = (MapNode) pair.getValue();
			numEdges += value.getNumEdges();

		}
		return numEdges;
	}

	
	
	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location)
	{

		if(graph.containsKey(location) || location == null){
			return false;
		}else{
			graph.put(location, new MapNode(location));
			return true;
		}
	}
	
	/**
	 * Adds a directed edge to the graph from pt1 to pt2.
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
						String roadType, double length) throws IllegalArgumentException {


		if(!graph.containsKey(from) || !graph.containsKey(to) || length == 0){
			throw new IllegalArgumentException("geographic points are null");
		}

		MapEdge edge = new MapEdge(graph.get(from), graph.get(to), roadName, roadType, length);
		graph.get(from).getAdjacentEdges().add(edge);
		graph.get(from).getNeighbours().add(graph.get(to));

	}
	

	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return bfs(start, goal, temp);
	}
	
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start,
									 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{

		// Hook for visualization.  See writeup.

		Queue<MapNode> queue = new LinkedList<MapNode>();
		HashSet<MapNode> visited = new HashSet<>();
		HashMap<MapNode, MapNode> parent = new HashMap<>();

		MapNode goalNode = graph.get(goal);
		queue.add(graph.get(start));
		visited.add(graph.get(start));

		while(!queue.isEmpty()){
			MapNode curr = queue.poll();
			nodeSearched.accept(curr.getLocation());
			if(curr.equals(goalNode)){
				return getPath(graph.get(start), goalNode, parent);
			}
			List<MapNode> neighbours = curr.getNeighbours();
			for(MapNode neighbour: neighbours){
				if(!visited.contains(neighbour)){
					visited.add(neighbour);
					parent.put(neighbour, curr);
					queue.add(neighbour);
				}
			}
		}


		return null;
	}

	private List<GeographicPoint> getPath(MapNode start, MapNode goal, HashMap<MapNode, MapNode> parent) {
		ArrayList<GeographicPoint> path = new ArrayList<>();
		MapNode curr = goal;
		while(true){
			if(curr.equals(start))
				break;
			MapNode next = parent.get(curr);
			path.add(curr.getLocation());
			curr = next;
		}

		path.add(start.getLocation());
		Collections.reverse(path);
		return path;
	}


	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start,
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		//initialize
		int count = 0;
		PriorityQueue<DistanceNode> priorityQueue = new PriorityQueue<>();
		HashSet<MapNode> visited = new HashSet<MapNode>();
		HashMap<MapNode, MapNode> parent = new HashMap<>();
		HashMap<MapNode, Double> distances = new HashMap<>();
		Integer inf = Integer.MAX_VALUE;

		//initialize all distances to infinity,
		for(GeographicPoint point: graph.keySet()){
			MapNode mapNode = graph.get(point);
			distances.put(mapNode, inf.doubleValue());
		}

		MapNode startNode = graph.get(start);
		MapNode goalNode = graph.get(goal);
		priorityQueue.add(new DistanceNode(startNode, 0.0));
		count++;
		distances.put(startNode, 0.0);

		while(!priorityQueue.isEmpty()){
			DistanceNode current = priorityQueue.poll();
			count++;
			nodeSearched.accept(current.getMapNode().getLocation());
			if(!visited.contains(current.getMapNode())){
				visited.add(current.getMapNode());
				if(current.getMapNode().getLocation().equals(goalNode.getLocation())) {
					System.out.println(count);
					return getPath(startNode, goalNode, parent);
				}
				else{
					for(MapEdge neighbouringEdge: current.getMapNode().getAdjacentEdges()){
						//if path from current to n is shorter then update n's distance
						Double alternate = distances.get(current.getMapNode()) + neighbouringEdge.start.getLocation().distance(neighbouringEdge.end.getLocation());
						if(alternate < distances.get(neighbouringEdge.end)){
							distances.put(neighbouringEdge.end, alternate);
							parent.put(neighbouringEdge.end, current.getMapNode());
							priorityQueue.add(new DistanceNode(neighbouringEdge.end, alternate));
						}
					}
				}
			}

		}
		
		return null;
	}

	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}
	
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start,
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{

		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());

		int count = 0;
		PriorityQueue<DistanceNode> priorityQueue = new PriorityQueue<>();
		HashSet<MapNode> visited = new HashSet<MapNode>();
		HashMap<MapNode, MapNode> parent = new HashMap<>();
		HashMap<MapNode, Double> distances = new HashMap<>();
		Integer inf = Integer.MAX_VALUE;

		//initialize all distances to infinity,
		for(GeographicPoint point: graph.keySet()){
			MapNode mapNode = graph.get(point);
			distances.put(mapNode, inf.doubleValue());
		}

		MapNode startNode = graph.get(start);
		MapNode goalNode = graph.get(goal);
		priorityQueue.add(new DistanceNode(startNode, 0.0, startNode.getLocation().distance(goalNode.getLocation())));
		count++;
		distances.put(startNode, 0.0);

		while(!priorityQueue.isEmpty()){
			DistanceNode current = priorityQueue.poll();
			count++;
			nodeSearched.accept(current.getMapNode().getLocation());
			if(!visited.contains(current.getMapNode())){
				visited.add(current.getMapNode());
				if(current.getMapNode().getLocation().equals(goalNode.getLocation())) {
					System.out.println(count);
					return getPath(startNode, goalNode, parent);
				}
				else{
					for(MapEdge neighbouringEdge: current.getMapNode().getAdjacentEdges()){
						//if path from current to n is shorter then update n's distance
						Double alternate = distances.get(current.getMapNode()) + neighbouringEdge.start.getLocation().distance(neighbouringEdge.end.getLocation());
						if(alternate < distances.get(neighbouringEdge.end)){
							distances.put(neighbouringEdge.end, alternate);
							parent.put(neighbouringEdge.end, current.getMapNode());
							priorityQueue.add(new DistanceNode(neighbouringEdge.end, alternate, neighbouringEdge.end.getLocation().distance(goalNode.getLocation())));
						}
					}
				}
			}

		}

		return null;
	}

	
	public static void main(String[] args)
	{
//		System.out.print("Making a new map...");
//		MapGraph theMap = new MapGraph();
//		System.out.print("DONE. \nLoading the map...");
//		GraphLoader.loadRoadMap("data/testdata/simpletest.map", theMap);
//		System.out.println("DONE.");

//		List<GeographicPoint> bfs = theMap.bfs();
//		System.out.println(bfs);

//		MapGraph theMap = new MapGraph();
//		System.out.print("DONE. \nLoading the map...");
//		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
//
//		System.out.println("DONE.");
//
//		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
//		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);


//		MapGraph theMap = new MapGraph();
//		System.out.print("DONE. \nLoading the map...");
//		GraphLoader.loadRoadMap("data/testdata/simpletest.map", theMap);
//
//		System.out.println("DONE.");
////
//		GeographicPoint start = new GeographicPoint(1, 1);
//		GeographicPoint end = new GeographicPoint(8, -1);
//
//		List<GeographicPoint> route = theMap.dijkstra(start,end);
//		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);
//
//		System.out.println(route);
//		System.out.println(route2);


		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);

		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);




	}
	
}
