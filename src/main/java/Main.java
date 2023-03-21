import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.time.Duration;
import java.time.LocalDate;
import java.time.LocalDateTime;
import java.time.LocalTime;
import java.time.temporal.ChronoUnit;
import java.util.*;
import java.util.stream.Collectors;

import static java.util.Arrays.asList;

public class Main {

    public static Map<String, Node> stopNameToNodeMap = new HashMap<>();
    public static Map<String, Set<String>> linesByStop = new HashMap<>();
    public static Map<String, List<Edge>> getEdgesByStartNode(LocalDateTime startTime) throws FileNotFoundException {
        List<Edge> edges = new ArrayList<>();
        Set<Node> nodes = new HashSet<>();
        try (Scanner scanner = new Scanner(new File("src/main/resources/connection_graph.csv"))) {
            scanner.nextLine(); // skip header
            while (scanner.hasNextLine()) {
                String line = scanner.nextLine();
                String[] tokens = line.split(",");
                String usedLine = tokens[3];
                LocalDateTime departureTime = LocalDateTime.of(LocalDate.now(), LocalTime.parse(tokens[4]));
                LocalDateTime arrivalTime = LocalDateTime.of(LocalDate.now(), LocalTime.parse(tokens[5]));
                if(departureTime.isBefore(startTime)) {
                    departureTime = departureTime.plusDays(1);
                    arrivalTime = arrivalTime.plusDays(1);
                }
                if(arrivalTime.isBefore(departureTime)) arrivalTime = arrivalTime.plusDays(1);
                String startStop = tokens[6];
                String endStop = tokens[7];
                Double startStopLon = Double.parseDouble(tokens[8]);
                Double startStopLat = Double.parseDouble(tokens[9]);
                Double endStopLat = Double.parseDouble(tokens[10]);
                Double endStopLon = Double.parseDouble(tokens[11]);
                nodes.add(new Node(startStop, startStopLon, startStopLat));
                nodes.add(new Node(endStop, endStopLon, endStopLat));
                edges.add(new Edge(startStop, endStop, usedLine, departureTime, arrivalTime, startStopLon, startStopLat, endStopLat, endStopLon));
            }
        }
        stopNameToNodeMap = nodes.stream().collect(Collectors.toMap(Node::getName, node -> node));
        var edgesByStartNode = edges.stream().collect(Collectors.groupingBy(Edge::getFrom));
        return edgesByStartNode;
    }
    public static void solveDijkstra(String start, String end, LocalDateTime startTime) throws FileNotFoundException {
        Map<String, List<Edge>> edgesByStartNode = getEdgesByStartNode(startTime);
        LocalTime startOfRunApp = LocalTime.now();
        Map<String, Long> distances = new HashMap<>();
        HashMap<String, Edge> traversedEdges = new HashMap<>();
        for (String stop : edgesByStartNode.keySet()) {
            distances.put(stop, Long.MAX_VALUE);
        }
        distances.put(start, 0L);
        PriorityQueue<String> pq = new PriorityQueue<>(Comparator.comparingLong(distances::get));
        pq.offer(start);
        while (!pq.isEmpty()) {
            String currentStop = pq.poll();
            if (currentStop.equals(end)) {
                break;
            }
            for (Edge edge : edgesByStartNode.get(currentStop)) {
                if (!edge.from.equals(currentStop)) {
                    continue;
                }
                String nextStop = edge.to;
                long newDistance = distances.get(currentStop) + edge.getDistanceByTime();
                if (newDistance < distances.get(nextStop)) {
                    distances.put(nextStop, newDistance);
                    traversedEdges.put(nextStop, edge);
                    pq.offer(nextStop);
                }
            }
        }
        System.out.println("Time of running program for Dijkstra in ms: " + startOfRunApp.until(LocalTime.now(), ChronoUnit.MILLIS));
        //path reconstruction
        List<Edge> path = new ArrayList<>();
        if (traversedEdges.containsKey(end)) {
            String currentStop = end;
            while (!currentStop.equals(start)) {
                path.add(traversedEdges.get(currentStop));
                currentStop = traversedEdges.get(currentStop).from;
            }
        }
        Collections.reverse(path);
        double shortestPathTime = distances.get(end);
        path.forEach(System.out::println);
        System.out.println("Total time of path in minutes:" + shortestPathTime);
    }

    private static double getCostFunctionBetweenTwoNodes(Node start, Node end){
        return Math.abs(start.lat- end.lat) + Math.abs(start.lon - end.lon);
    }

    private static double getCostFunctionForAStar(double startingCost, Node start, Node intermediary, Node end){
        return getCostFunctionBetweenTwoNodes(intermediary, end) + startingCost;
    }

    public static List<Edge> reconstructPath(Map<String, Edge> cameFrom, String stop){
        List<Edge> totalPath = new ArrayList<>();
        while (cameFrom.containsKey(stop)){
            totalPath.add(cameFrom.get(stop));
            stop = cameFrom.get(stop).getFrom();
        }
        Collections.reverse(totalPath);
        totalPath.forEach(System.out::println);
        long sumTime = totalPath.stream().mapToLong(edge -> Duration.between(edge.departureTime, edge.arrivalTime).toMinutes()).sum();
        System.out.println("Total time of path in minutes: " + sumTime);
        return totalPath;
    }

    public static List<Edge> solveAStarTimeDependent(String start, String end, LocalDateTime startTime, Map<String, List<Edge>> edgesByStartNode) throws FileNotFoundException {
        LocalTime startOfRunApp = LocalTime.now();
        Node endNode = stopNameToNodeMap.get(end);
        Set<Node> openNodes = new HashSet<>();
        openNodes.add(stopNameToNodeMap.get(start));
        Set<Node> closedNodes = new HashSet<>();
        Map<Node, Double> costOfGoingToNode = new HashMap<>(Map.of(stopNameToNodeMap.get(start), 0d));
        Map<String, Edge> cameFrom = new HashMap<>();
        Map<Node, Double> minimizedCostFunctionValueMap = new HashMap<>(Map.of(stopNameToNodeMap.get(start), 0d));
        while (!openNodes.isEmpty()){
            Node node = null;
            double nodeCost = Double.MAX_VALUE;
            for (Node testNode : openNodes){
                if (minimizedCostFunctionValueMap.get(testNode)<nodeCost){
                    node = testNode;
                    nodeCost = minimizedCostFunctionValueMap.get(testNode);
                }
            }
            openNodes.remove(node);
            closedNodes.add(node);
            if (node.equals(endNode)) {
                System.out.println("Time of running program for A* in ms: " + startOfRunApp.until(LocalTime.now(), ChronoUnit.MILLIS));
                return reconstructPath(cameFrom, endNode.getName());
            }
            for (Edge edge : edgesByStartNode.get(node.name)){
                Node observedNode = stopNameToNodeMap.get(edge.to);
                if (!openNodes.contains(observedNode) && !closedNodes.contains(observedNode)){
                    openNodes.add(observedNode);
                    cameFrom.put(observedNode.name, edge);
                    costOfGoingToNode.put(observedNode, edge.getDistanceByTime()+ costOfGoingToNode.get(node));
                    minimizedCostFunctionValueMap.put(observedNode, getCostFunctionForAStar(costOfGoingToNode.get(node), node, observedNode, endNode));
                }
                else if (costOfGoingToNode.get(observedNode)> edge.getDistanceByTime()+ costOfGoingToNode.get(node)){
                    costOfGoingToNode.put(observedNode, edge.getDistanceByTime()+ costOfGoingToNode.get(node));
                    minimizedCostFunctionValueMap.put(observedNode, getCostFunctionForAStar(costOfGoingToNode.get(node), node, observedNode, endNode));
                    cameFrom.put(observedNode.name, edge);
                    if (closedNodes.contains(observedNode)){
                        openNodes.add(observedNode);
                        closedNodes.remove(observedNode);
                    }
                }
            }
        }
        return null;
    }

    public static void solveAStarTransferDependent(String start, String end, LocalDateTime startTime) throws FileNotFoundException {
        Map<String, List<Edge>> edgesByStartNode = getEdgesByStartNode(startTime);
        LocalTime startOfRunApp = LocalTime.now();
        Node endNode = stopNameToNodeMap.get(end);
        Set<Node> openNodes = new HashSet<>();
        openNodes.add(stopNameToNodeMap.get(start));
        Set<Node> closedNodes = new HashSet<>();
        Map<Node, Double> costOfGoingToNode = new HashMap<>(Map.of(stopNameToNodeMap.get(start), 0d));
        Map<String, Edge> cameFrom = new HashMap<>();
        Map<Node, Double> minimizedCostFunctionValueMap = new HashMap<>(Map.of(stopNameToNodeMap.get(start), 0d));
        while (!openNodes.isEmpty()){
            Node node = null;
            double nodeCost = Double.MAX_VALUE;
            for (Node testNode : openNodes){
                if (minimizedCostFunctionValueMap.get(testNode)<nodeCost){
                    node = testNode;
                    nodeCost = minimizedCostFunctionValueMap.get(testNode);
                }
            }
            openNodes.remove(node);
            closedNodes.add(node);
            if (node.equals(endNode)) {
                System.out.println("Time of running program for A* in ms: " + startOfRunApp.until(LocalTime.now(), ChronoUnit.MILLIS));
                reconstructPath(cameFrom, endNode.getName());
                return;
            }
            for (Edge edge : edgesByStartNode.get(node.name)){
                Node observedNode = stopNameToNodeMap.get(edge.to);
                String previousLine = Optional.ofNullable(cameFrom.get(edge.from)).map(e -> e.usedLine).orElse("");
                Double costOfGoingBetweenStops = edge.getUsedLine().equals(previousLine) ? 0d : 0.045d;
                Double costFunctionRes = getCostFunctionForAStar(costOfGoingToNode.get(node), node, observedNode, endNode);
                if (!openNodes.contains(observedNode) && !closedNodes.contains(observedNode)){
                    openNodes.add(observedNode);
                    cameFrom.put(observedNode.name, edge);
                    costOfGoingToNode.put(observedNode,  costOfGoingBetweenStops+ costOfGoingToNode.get(node));
                    minimizedCostFunctionValueMap.put(observedNode, costFunctionRes);
                }
                else if (costOfGoingToNode.get(observedNode)> costOfGoingBetweenStops+ costOfGoingToNode.get(node)){
                    costOfGoingToNode.put(observedNode, costOfGoingBetweenStops + costOfGoingToNode.get(node));
                    minimizedCostFunctionValueMap.put(observedNode, costFunctionRes);
                    cameFrom.put(observedNode.name, edge);
                    if (closedNodes.contains(observedNode)){
                        openNodes.add(observedNode);
                        closedNodes.remove(observedNode);
                    }
                }
            }
        }
    }

    public static void solveAStar(String start, String end, char optimizationCriterion, LocalDateTime startTime) throws FileNotFoundException {
        Map<String, List<Edge>> edgesByStartNode = getEdgesByStartNode(startTime);
        if (optimizationCriterion =='t') solveAStarTimeDependent(start, end, LocalDateTime.now(), edgesByStartNode);
        else solveAStarTransferDependent(start, end, LocalDateTime.now());
    }

    private static List<Edge> generateInitialSolutionForTabuSearch(String start, List<String> stopsToVisit, Map<String, List<Edge>> edgesByStartNode) throws FileNotFoundException {
        String currentNode = start;
        List<Edge> initialSolution = new ArrayList<>();
        for (int i = 0; i < stopsToVisit.size(); i++) {
            initialSolution.addAll(solveAStarTimeDependent(currentNode, stopsToVisit.get(i), LocalDateTime.now(), edgesByStartNode));
            currentNode =stopsToVisit.get(i);
        }
        initialSolution.addAll(solveAStarTimeDependent(currentNode, start, LocalDateTime.now(), edgesByStartNode));
        return initialSolution;
    }

    public static void solveTabu(String start, List<String> stopsToVisit, char optimizationCriterion, LocalDateTime startTime) throws FileNotFoundException {
        Map<String, List<Edge>> edgesByStartNode = getEdgesByStartNode(startTime);
        List<Edge> initialSolution= generateInitialSolutionForTabuSearch(start, stopsToVisit, edgesByStartNode);
        List<Edge> currentSolution= new ArrayList<>(initialSolution);
        List<Edge> bestKnownSolution = new ArrayList<>(currentSolution);
        Set<Node> tabuSet = new HashSet<>();
        List<Edge> neighborhood;
        while (tabuSet.size()<stopsToVisit.size()*10){
            neighborhood = currentSolution.stream().map(edge -> edgesByStartNode.get(edge.getFrom())).flatMap(List::stream).collect(Collectors.toList());
            for(Edge edge : neighborhood){
                if (!tabuSet.contains(edge.to)){
                    currentSolution.add(edge);
                }
                neighborhood.forEach(edge1 -> tabuSet.add(stopNameToNodeMap.get(edge1.to)));
            }
            if (currentSolution.stream().mapToDouble(edge -> edge.getDistanceByTime()).sum() < bestKnownSolution.stream().mapToDouble(edge -> edge.getDistanceByTime()).sum()){
                bestKnownSolution = new ArrayList<>(currentSolution);
            }
        }
        System.out.println("Current solution for tabu");
        bestKnownSolution.forEach(System.out::println);
    }

    public static void main(String[] args) throws IOException {
//        solveDijkstra("Tramwajowa", "PL. GRUNWALDZKI", LocalDateTime.now());
//        solveAStar("Tramwajowa", "PL. GRUNWALDZKI", 'p', LocalDateTime.now());
        solveTabu("Tramwajowa", List.of("PL. GRUNWALDZKI", "GALERIA DOMINIKAŃSKA"), 't', LocalDateTime.now());
    }
}
