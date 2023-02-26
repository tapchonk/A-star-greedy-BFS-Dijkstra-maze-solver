
/*
* File: GrandFinale.java
* Created: 05/12/2021
* Author: 2145461
*Preamble: This solution does not view the maze as a collection of nodes but rather as a series of "conveyer belts" all leading to the root node. This approach rank orders new nodes it comes across
*in a priority queue as specified in the priorityNode class whilst every other node is stored in a hashmap. The way in which we can access data within our hashmap is based on a sort of cheat we can use
*between the key and value data in our hashmap. It leads from the fact that ArrayList objects in java all have the same hash value if the values contained in the array list are the same. This is an
*incredibly useful "trick" we can use due to the following: we can set the key for our hashmap data as an array list containing the (x, y) coordinate of the node we want to store data about and the values
*associated with that key as any data we want to store about our node. This data could be anything such as arrived from directions, an estimated heuristic distance from the node to our target location
*or even the nodes cost to get to that node. In this case, I used it to store an array list object that contains the (x, y) coordinate of our parent node.The main benefit of using a
*hashmap to store our data is simply the fact that access times for our data is extremely fast as it is O(1). Now, some problems do exist with this solution,
*one of the most prominent issues with this method of storing essentially every node in memory is quite clearly the fact that we are going to use a lot of memory in this case to store information about every node.
*Also, whilst the access times are initially much faster than that of an array that searches for elements sequentially after a certain point once we start adding more and more elements to our hash table then the
*worst case hashmap acces time is O(n).
*More notes about the priority queue: We only use an array to store nodes on our "frontier" or "fringe" and the array is sorted such that the cost of the node is based on two factors: the cost of getting to the node
*from the root node and an estimate of the total distance remaining between the current node and out target location.
*I have tested this design on all the maze generators on the maximum size multiple times and in nearly all cases the robot can produce an optimal path between the root and the target. It will always complete the maze.
*The node cost is determined by f(n) = h(n) + g(n) and the nodes in the priority queue are sorted from lowest to highest such that nodes with a lower combined g(n) and h(n) are higher priority and are expanded on first.
*However, only storing information about the parent node and not including information pertaining vertexes/connections with other edges means that we are using tree structured data. How are we going to find fastest paths
*and expand on other nodes if we are limited by tree structured data? In the class "MazeTree" we have a method that allows us to construct a path between any two nodes on our tree in the form of a 2D array
*containing the (x, y) coordinates of the nodes that the robot has to travel through in order to reach the node that we want to expand. This method is denoted by createTraverseArray(x1, y1, x2, y2). This method
*assumes the fact that all data converges at the root node, meaning that if you were to find the parent node of any coordinate and repeat the process n times then we will always reach the root node meaning
*that a connection "exists" between any two nodes on our tree in an indirect manner. This allows us to turn any tree structured data into a sort of "pseudo-graph-like" structure as every node does in fact have a vertex
*connecting us to every other node except it may be more "indirect" than we would like. This means that the fastest path to expand a node may not be found but an optimal path can be found between the root and the target.
*The method that this entire controller relies on is a way of constructing paths between nodes on our tree(s) (Essentially we are storing multiple trees to represent each path that the robot could diverge on)
*The method has 2 steps that are essential to getting this working and another step at the end to compress this path and save memory on our second run.
*1.) build a path from the target to the root and the origin to the root and find the first coordinate that they have in common.
*2.) construct a 2D array from the origin node to the common coordinate and then from the common coordinate to the target node. (this is a tree traversal where the robot visits nodes on its own tree exactly once)
*3.) if the robot has completed the maze we construct a path from the root to the end and can compress our final path using a lossless compression algorithm (we identify repeated steps and combine them into one big step).
*Whilst yes, we are expanding the paths we are generating node by node which does take significantly longer, it guarantees that we can find an optimal path to the target on our second run.
*Also, by using a heuristic estimate we can reduce the number of nodes expanded which can save memory space and ensures that we can find an optimal path more quickly.
*On the other hand, if the optimal path to a target is not close to a straight line then this is where the algorithm falls flat as it essentially devolves into BFS.
*Considering how we can now construct a path from any node on our tree to any other node on our tree. We can essentially change the target to be any node that the robot has previously explored assuming that it has explored there before.
*(except the map gets reset whenever you change something about it so that's pretty pointless).
*You can also change the behaviour of the robot so that it runs more like Dijkstra's or Greedy Best First Search.
*All you have to do is change the values of the hMultiplier and gCostMultiplier variables. (lines 98, 99, 124, 125)
*Set the gCostMultiplier to 0 and hMultiplier to 1 for Greedy Best First Search.
*Set the gCostMultiplier to 1 and hMultiplier to 0 for Dijkstra's-like behaviour. (it's interesting to watch this behaviour on a max size prim generation)
*This solution is significantly better than the other two provided by the guide mainly due to two reasons: reduces total number of searched nodes by using a heuristic and secondly we are able to find the optimal path on any maze and solve it instantaneously on its second run.
*/
import java.util.Arrays; //some array libraries
import java.util.ArrayList;
import java.util.List;
import java.util.HashMap; //hashmap library
import uk.ac.warwick.dcs.maze.logic.IRobot; //the warwick IRobot interface library

public class GrandFinale {
	private int pollRun = 0; // these are some privately declared variables I use to keep track of data
								// during (and after) a run
	private int whichCoord = 0;
	private PriorityNode priorityNode; // priority node object
	private MazeTree node; // MazeTree object
	private double hMultiplier;
	private int[][] traversalArray;
	private int[][] finalPath;

	public void reset() // reset our poll run on resets of the maze
	{
		node = new MazeTree(0, 0, 0, 0); // declare our new MazeTree and priorityNode objects
		priorityNode = new PriorityNode(0, 0, 0);
		pollRun = 0;
		traversalArray = new int[0][2]; // resets the traverse array which allows the robot to visit any two nodes on
										// its tree
		whichCoord = 0; // resets the cooordinate selector variable
	}

	public void controlRobot(IRobot robot) // main control method
	{
		// if (pollRun % 1000 == 0) System.out.println("priority length " +
		// ignore.getPriorityQueue().length);
		// System.out.println(Arrays.deepToString(traversalArray));
		int x1 = robot.getLocation().x;
		int y1 = robot.getLocation().y;

		if ((robot.getRuns() == 0) && (pollRun == 0))
			initialiseBot(robot); // at the start of a fresh maze we initialise our arrays
		if ((robot.getRuns() == 0) && (pollRun > 0))
			forcePriorityNodeTraversal(robot); // Calls our traversal method to travel to a node based on the next node
												// in our priority queue and follow the instructions to get there
		if (robot.getRuns() != 0 && pollRun >= 0)
			pathFoundController(robot); // when we have completed a maze for the first time we tell it to follow the
										// instructions set out to travel on the fastest path

		int targetX = robot.getTargetLocation().x; // gets the location of our target
		int targetY = robot.getTargetLocation().y;
		if (((x1 >= targetX - 1 && x1 <= targetX + 1) & (y1 == targetY))
				| ((x1 == targetX) & (y1 >= targetY - 1 && y1 <= targetY + 1))) {
			if (robot.getRuns() == 0)
				checkDone(robot); // when we are near our target we call the checkDone method
		}

		pollRun++; // increment pollRun
	}

	private void initialiseBot(IRobot robot) // some initialization parameters to reset our data store and tell the
												// robot how we want to search through our maze
	{
		int x1 = robot.getLocation().x;
		int y1 = robot.getLocation().y;

		node = new MazeTree(0, 0, 0, 0); // reset our MazeTree data store
		priorityNode = new PriorityNode(0, 0, 0); // reset our priority queue data store
		MazeTree node1 = new MazeTree(5000077, 5000077, x1, y1); // declares our root node. We can identify it as our
																	// root node as if you get the parent for this node
																	// then it should return 5000077 (it's prime)
		PriorityNode priorityNode1 = new PriorityNode(x1, y1, 5000077); // declares our first priority node

		node.setGCostMultiplier(1); // you can make the search "greedy" by setting the G Cost Multiplier to 0
		hMultiplier = Math.sqrt(2); // you can make the robot act like Dijkstra's by setting the Heuristic
									// Multiplier to 0

		traversalArray = new int[0][2]; // reset our traverse array

		node.resetCoordinateArray();
		priorityNode.resetQueueArray();

		node.AddNode(node1); // adds our root node to the MazeTree hashmap
		priorityNodeQueuer(robot, 0); // queues the first set of priority nodes surrounding the robot

		int targetX = priorityNode.getPriorityQueue()[0].getX();
		int targetY = priorityNode.getPriorityQueue()[0].getY();
		traversalArray = node.createTraverseArray(x1, y1, targetX, targetY);
		forcePriorityNodeTraversal(robot); // runs the robot to follow the expansion path so every path we expand is
											// precalculated based on the data in our priorty queue and the path
											// constructed by "createTraverseArray"
	}

	private void forcePriorityNodeTraversal(IRobot robot) {
		int x1 = robot.getLocation().x; // predeclaring our coordinate values
		int y1 = robot.getLocation().y;
		int x2 = 0;
		int y2 = 0;

		int pos = wallCounter(robot); // int variable for the switch cases to check if it's in a corridor or junction

		switch (pos) {
			case 1:
				node.setGCostMultiplier(1);
				hMultiplier = Math.sqrt(2); // set multipliers back to default
				break;
			case 2:
				node.setGCostMultiplier(0.01); // we do this in order for the robot to expand corridors more quickly as
												// opposed to using a messier solution involving
				hMultiplier = Math.sqrt(0.01); // set the multipliers really low for corridors
		}

		int arrayX = traversalArray[traversalArray.length - 1][0]; // once we have expanded on a node we look for a new
																	// node to expand upon
		int arrayY = traversalArray[traversalArray.length - 1][1]; // So essentially once the robot has reached its
																	// short term target we tell the robot to construct
																	// a new path and reset our whichCoord selector
																	// variable

		if (traversalArray.length != 0 && x1 == arrayX && y1 == arrayY) {
			priorityNode.removeTraversedNode(); // remove the priority node that we expanded on
			priorityNodeQueuer(robot, node.gCostCalculator(x1, y1)); // queue up a new set of nodes for the robot in the
																		// priority queue
			int targetX = priorityNode.getPriorityQueue()[0].getX();
			int targetY = priorityNode.getPriorityQueue()[0].getY();
			traversalArray = node.createTraverseArray(x1, y1, targetX, targetY); // calls our function to create a new
																					// path to expand on a new node

			whichCoord = 0; // set our target as the first element in our traversal array once we have
							// expanded a node
		}

		x2 = traversalArray[whichCoord][0]; // we set the target for the robot as the elements within our traversalArray
											// based on the iteration of our whichCoord variable
		y2 = traversalArray[whichCoord][1];

		if (x1 == x2 && y1 == y2) // when our current location is the node in our traversalArray we increment the
									// whichCoord variable so that our target is now the next element in the
									// traversalArray
		{
			if (whichCoord < traversalArray.length - 1) {
				whichCoord++; // increment global variable "whichCoord"

				x2 = traversalArray[whichCoord][0];
				y2 = traversalArray[whichCoord][1];
			}
		}

		int attemptDirection = createDirectionArray(robot, x1, y1, x2, y2); // A method to choose which direction to go
																			// to in order to reach our target location
																			// which is the nodes in the traversal array
		robot.setHeading(attemptDirection); // set the heading of the robot

	}

	private void pathFoundController(IRobot robot) // This method only runs after the robot's first run and tells the
													// robot to follow the path set out for it in the finalPath array
	{
		int x1 = robot.getLocation().x; // coordinate variables
		int y1 = robot.getLocation().y;
		int x2 = 0;
		int y2 = 0;

		x2 = finalPath[whichCoord][0]; // (sets our x, y) target coordinates
		y2 = finalPath[whichCoord][1];

		if (x1 == x2 && y1 == y2 & whichCoord < finalPath.length) // iterate through the coordinates in the array as
																	// long as we have reached a target (or are at least
																	// near it)
		{
			whichCoord++;
			if (whichCoord < finalPath.length) {
				x2 = finalPath[whichCoord][0];
				y2 = finalPath[whichCoord][1]; // the target is the coordinates in our array coined "finalPath"
			}
		}

		int attemptDirection = createDirectionArray(robot, x1, y1, x2, y2); // chooses a direction to our short term
																			// target
		robot.setHeading(attemptDirection); // sets the heading to that direction
	}

	private void queueNode(IRobot robot, int x1, int y1, int x2, int y2, double gCost) // method to add node information
																						// and queue priority nodes
	{
		MazeTree nodeNew = new MazeTree(x1, y1, x2, y2); // declares a new MazeTree object variable
		node.AddNode(nodeNew); // adds the new object to our hashmap

		double heuristic = absoluteMagnitude(x2, y2, robot.getTargetLocation().x, robot.getTargetLocation().y); // calculates
																												// the
																												// heuristic
																												// estimate
																												// to
																												// our
																												// target
																												// from
																												// the
																												// current
																												// node
		PriorityNode priorityNodeNew = new PriorityNode(x2, y2, heuristic * hMultiplier + gCost); // queues up the
																									// priority node
																									// with values for
																									// gCost and the
																									// heuristic
																									// estimate
		priorityNode.addPriorityNode(priorityNodeNew); // adds the priority node
	}

	private void priorityNodeQueuer(IRobot robot, double gCost) // every time we expand on a node we check surrounding
																// nodes and determine what position in the priority
																// queue the nodes should be in
	{
		int x1 = robot.getLocation().x;
		int y1 = robot.getLocation().y;

		if (robot.look(lookHeading(robot, IRobot.NORTH)) != IRobot.BEENBEFORE
				&& robot.look(lookHeading(robot, IRobot.NORTH)) != IRobot.WALL) // for every 4 squares surrounding the
																				// robot we queue up priority nodes
			queueNode(robot, x1, y1, x1, y1 - 1, gCost); // calls method to queue up priority node to the north of the
															// robot

		if (robot.look(lookHeading(robot, IRobot.EAST)) != IRobot.BEENBEFORE
				&& robot.look(lookHeading(robot, IRobot.EAST)) != IRobot.WALL)
			queueNode(robot, x1, y1, x1 + 1, y1, gCost); // calls method to queue up priority node to the east of the
															// robot

		if (robot.look(lookHeading(robot, IRobot.SOUTH)) != IRobot.BEENBEFORE
				&& robot.look(lookHeading(robot, IRobot.SOUTH)) != IRobot.WALL)
			queueNode(robot, x1, y1, x1, y1 + 1, gCost); // calls method to queue up priority node to the south of the
															// robot

		if (robot.look(lookHeading(robot, IRobot.WEST)) != IRobot.BEENBEFORE
				&& robot.look(lookHeading(robot, IRobot.WEST)) != IRobot.WALL)
			queueNode(robot, x1, y1, x1 - 1, y1, gCost); // calls method to queue up priority node to the west of the
															// robot

	}

	private int lookHeading(IRobot robot, int currentHeading) // turns absolute headings to relative headings
	{
		int whereToLook = IRobot.AHEAD + ((currentHeading - robot.getHeading() + 4) % 4);

		return whereToLook; // returning our object that we are looking at
	}

	private int wallCounter(IRobot robot) // Rather than counting the number of passages we count the number of walls.
											// It's essentially the same thing.
	{
		int wallNumber = 0;
		int whereLook = IRobot.AHEAD;
		while (whereLook <= IRobot.LEFT) // while loop to check what's around the robot
		{
			if (robot.look(whereLook) == IRobot.WALL) // if we are looking at a wall then we increment our wallNumber
														// variable by one
			{
				wallNumber++;
			}
			whereLook++; // change which direction we are looking
		}
		if (wallNumber == 0)
			wallNumber++; // junctions are the same as crossroads so we don't need to account for a fourth
							// state

		return wallNumber; // return the number of walls we counted
	}

	private void checkDone(IRobot robot) // when the robot is next to the target we do the following:
	{
		whichCoord = 0; // reset our "whichCoord" coordinate selector variable
		int x1 = robot.getLocation().x; // gets the location of the robot and the target
		int y1 = robot.getLocation().y;
		int x2 = robot.getTargetLocation().x;
		int y2 = robot.getTargetLocation().y;
		int targetX = node.returnCoordinateArray()[0].getX();
		int targetY = node.returnCoordinateArray()[0].getY();
		finalPath = node.createTraverseArray(targetX, targetY, x1, y1); // calls to create a path from the root node to
																		// the target location
		finalPath = node.losslessCompressionAlgorithm(finalPath, x2, y2); // runs the lossless compression algorithm to
																			// reduce the final path size
		System.out.println("Optimal path found: " + Arrays.deepToString(finalPath)); // outputs the nodes the robot has
																						// to go through in order to
																						// complete the maze

		int attemptDirection = createDirectionArray(robot, x1, y1, x2, y2); // home in on the target location
		robot.setHeading(attemptDirection); // set the direction

	}

	private double absoluteMagnitude(int x1, int y1, int x2, int y2) // method to calculate the euclidean distance
																		// between 2 sets of (x, y) coordinates
	{
		double xdif = Math.abs(x2 - x1);
		double ydif = Math.abs(y2 - y1);
		double absMag = Math.sqrt(xdif * xdif + ydif * ydif);
		return absMag; // return the euclidean distance as a double
	}

	private int createDirectionArray(IRobot robot, int x1, int y1, int x2, int y2) // method to choose the best
																					// direction to go in order to det
																					// closer to any target set of
																					// coordinates given the robots
																					// current position
	{
		double absoluteEast = absoluteMagnitude(x1 + 1, y1, x2, y2); // check euclidean distance east
		double absoluteWest = absoluteMagnitude(x1 - 1, y1, x2, y2); // check euclidean distance west
		double absoluteNorth = absoluteMagnitude(x1, y1 - 1, x2, y2); // check euclidean distance north
		double absoluteSouth = absoluteMagnitude(x1, y1 + 1, x2, y2); // check euclidean distance south
		// the absolute mangnitude between two points in 2D space is sqrt(Xbar^2 +
		// Ybar^2)
		// System.out.println("South" + absolutesouth);
		double[] absoluteArray = { absoluteNorth, absoluteEast, absoluteWest, absoluteSouth }; // sorts the array from
																								// smallest to largest
																								// magnitude from the
																								// target
		Arrays.sort(absoluteArray); // sorts the array from closest calculated direction heuristic to the furthest
		// System.out.println("Shortest distance: " + absoluteArray[0]);
		int attemptDirection = 0;
		if (absoluteNorth == absoluteArray[0]) // depending on which shortest euclidean distance matches to which
												// associated directions distance we return back the direction that will
												// bring us closest to the target coordinates
			attemptDirection = IRobot.NORTH;
		else if (absoluteEast == absoluteArray[0])
			attemptDirection = IRobot.EAST;
		else if (absoluteWest == absoluteArray[0])
			attemptDirection = IRobot.WEST;
		else if (absoluteSouth == absoluteArray[0])
			attemptDirection = IRobot.SOUTH;

		return attemptDirection; // returns the best direction (to reach a target)
	}
}

class MazeTree // this class is used mainly for the storage of paths from any node to any other
				// node assuming the robot has explored there
{
	public MazeTree(int parentX, int parentY, int x, int y) // this is our MazeTree object which I use to just pass
															// value between one class to another (I don't actually use
															// this to store any data as I found that the hashmap was a
															// better implementation)
	{
		this.parentX = parentX;
		this.parentY = parentY;
		this.x = x;
		this.y = y;
	}

	private int parentX; // how this tree structure works in general is that every node points to its
							// parent node and every node will eventually point back to the root node
							// (starting node)
	private int parentY;
	private int x;
	private int y;
	private double gCostMultiplier; // this is a static variable to keep our gCost multiplier the same but we can
									// change it if we want to to change the behaviour of our search algorithm
	private MazeTree[] coordinateArray;
	private HashMap<List<Integer>, List<Integer>> fasterParent; // hashmap containing information about the nodes (x, y)
																// coordinate and their associated parent node

	public void setGCostMultiplier(double gCostInput) {
		gCostMultiplier = gCostInput; // sets our gCostMultiplier accordinagly
	}

	public MazeTree[] returnCoordinateArray() {
		return coordinateArray; // return the array containing the coordinate of our root
	}

	public int getParentY() {
		return parentY; // gets the parent node X coordinate of a MazeTree object
	}

	public int getParentX() {
		return parentX; // gets the parent node Y coordinate of a MazeTree object
	}

	public int getY() {
		return y; // returns a nodes Y coordinate from a MazeTree object
	}

	public int getX() {
		return x; // returns a nodes X coordinate from a MazeTree object
	}

	public void resetCoordinateArray() // calls to reset our hashmap and the coordinate array
	{
		coordinateArray = new MazeTree[0];
		fasterParent = new HashMap<List<Integer>, List<Integer>>();
	}

	public void AddNode(MazeTree node) // method to add a node to our hashmap
	{

		List<Integer> coord = new ArrayList<Integer>(); // declare an array list that will contain the (x, y) coordinate
														// of the node
		List<Integer> parentCoord = new ArrayList<Integer>(); // declare an array list that will contain the (x, y)
																// coordinate of the parent of that node
		coord.add(node.getX()); // adds the (x, y) coordinate to our array list object
		coord.add(node.getY());
		parentCoord.add(node.getParentX()); // adds the (x, y) coordinate of our parent node to our array list object
		parentCoord.add(node.getParentY());
		if (fasterParent.containsKey(coord) != true) // we check if the key for the array list containing the (x, y)
														// coordinate of the current node is in the hashmap already
		{
			fasterParent.put(coord, parentCoord);
		}

		if (coordinateArray.length == 0) // this is just to store the root node
		{
			MazeTree[] newArray = new MazeTree[1];
			newArray[0] = node;
			coordinateArray = newArray;
		}

	}

	private int[] getParent(int x1, int y1) // method to get the parent coordinates of the parent node
	{
		List<Integer> coord = new ArrayList<Integer>(); // declares a new array list that will contain the (x, y)
														// coordinate of the hash key we're looking for

		coord.add(x1);
		coord.add(y1);
		int[] parentArray = new int[2]; // declares an array that we will pass back containingthe parent node
										// coordinates
		int arrayLength = coordinateArray.length;

		if (fasterParent.get(coord) != null) // we check if the coordinate we're trying to access with our hash value is
												// empty or not
		{
			parentArray[0] = fasterParent.get(coord).get(0); // this may seem confusing since the .get() method is the
																// same for array lists and hashmaps but there's no
																// avoiding it
			parentArray[1] = fasterParent.get(coord).get(1); // so firstly we are getting the hashmap value associated
																// with our given hash key (the array list object) and
																// afterwards we get the (x, y) coordinate from the
																// array list
		}

		return parentArray; // return an array containing the parent coordinate of any given node
	}

	private int[][] addArrayToArray(int arr[][], int arrAdd[]) // a method to add an array to a 2D array
	{
		int oldArrayLength = arr.length;
		int tempArr[][] = new int[oldArrayLength + 1][2]; // declares a new 2D array with an extra space for our array
															// we're adding

		for (int i7 = 0; i7 < oldArrayLength; i7++) // duplicates our 2D array over to a temporary array
			tempArr[i7] = arr[i7];

		tempArr[oldArrayLength] = arrAdd; // sets the last space in the array as the array we're adding
		return tempArr;
	}

	public double gCostCalculator(int x1, int y1) // calculates the gCost value for any given node given any (x, y)
													// coordinate
	{
		double gCost = 0;
		int startX = x1; // declares the starting x, y coordinate and the parent x, y coordinate
		int startY = y1;
		int parentX = 0;
		int parentY = 0;
		int i6 = 0;

		while (i6 < fasterParent.size() && parentX != 5000077) // while our count variable is less than the hashmap size
																// and the current node we are looking at is not the
																// root node
		{
			parentX = getParent(startX, startY)[0]; // gets the parent (x, y) coordinate of our node
			parentY = getParent(startX, startY)[1];

			if (parentX != 5000077)
				gCost = gCost + java.lang.Math.abs(parentX - startX) + java.lang.Math.abs(parentY - startY); // calculates
																												// the
																												// distance
																												// between
																												// each
																												// node
																												// and
																												// adds
																												// it to
																												// oue
																												// total
																												// gCost

			startX = parentX; // sets our starting node as the parent coordinate
			startY = parentY;
			i6++; // increments the i6 count variable
		}

		gCost = (gCost * gCostMultiplier); // scales our final gCost value with our gCost multiplier

		return gCost; // return gCost
	}

	public int[][] createTraverseArray(int x1, int y1, int x2, int y2) // this method is a bit of a doozy but what it
																		// essentially does is construct a path from any
																		// node on our tree to any other node on our
																		// tree
	{
		// System.out.println(" x1 " + x1 + " y1 " + y1);
		// System.out.println(" x2 " + x2 + " y2 " + y2);
		int[][] traverseArray = new int[0][2]; // we declare three arrays
		int[][] originArray = new int[0][2]; // the traverse array is our final output array
		int[][] targetArray = new int[0][2]; // the other two are arrays we construct from a set of coordinates to
												// (possibly) the root node

		int originX1 = x1; // this declares our starting node as the x, y coordinate pertaining to our
							// start
		int originY1 = y1;
		int originParentX; // for the parent of our current node
		int originParentY;

		int targetX1 = x2; // this declares the x, y coordinate pertaining to our end node
		int targetY1 = y2;
		int targetParentX; // for the parent of our target node
		int targetParentY;

		int commonX = 0;
		int commonY = 0;

		boolean commonCoordinateFound = false; // boolean true/false variable to see if we have found the common
												// coordinate and to prematurely stop our loops once we have identified
												// the common coordinate on our paths
		int i5 = 0;

		HashMap<List<Integer>, Integer> dupeCheck = new HashMap<List<Integer>, Integer>();

		while (commonCoordinateFound == false & i5 < fasterParent.size()) // while our counter is less than the hashmap
																			// size and we have not yet found the common
																			// coordinate do the following:
		{
			int[] originSingleCoordinateArray = new int[2]; // declare an array that will contain the (x, y) coordinate
															// of the origin coordinate we are comparing to the rest of
															// the target array
			int[] targetSingleCoordinateArray = new int[2]; // declare an array that will contain the (x, y) coordinate
															// of the target coordinate we are comparing to the rest of
															// the origin array

			originSingleCoordinateArray[0] = originX1;
			originSingleCoordinateArray[1] = originY1;

			targetSingleCoordinateArray[0] = targetX1;
			targetSingleCoordinateArray[1] = targetY1;

			List<Integer> coord1 = new ArrayList<Integer>();
			List<Integer> coord2 = new ArrayList<Integer>();

			if (originSingleCoordinateArray[0] != 0 && originSingleCoordinateArray[1] != 0) // we add the (x, y)
																							// coordinate to our origin
																							// array or target array as
																							// long as they are not
																							// blank (equal to 0)
			{
				coord1.add(originX1);
				coord1.add(originY1);
				if (dupeCheck.containsKey(coord1)) {
					commonCoordinateFound = true;
					commonX = originX1;
					commonY = originY1;
				} else {
					dupeCheck.put(coord1, 0);
				}
				originArray = addArrayToArray(originArray, originSingleCoordinateArray);
			}
			if (targetSingleCoordinateArray[0] != 0 && targetSingleCoordinateArray[1] != 0) {
				coord2.add(targetX1);
				coord2.add(targetY1);
				if (dupeCheck.containsKey(coord2)) {
					commonCoordinateFound = true;
					commonX = targetX1;
					commonY = targetY1;
				} else {
					dupeCheck.put(coord2, 0);
				}
				targetArray = addArrayToArray(targetArray, targetSingleCoordinateArray);
			}

			originParentX = getParent(originX1, originY1)[0]; // gets the parent of our node from the origin array
			originParentY = getParent(originX1, originY1)[1];

			targetParentX = getParent(targetX1, targetY1)[0]; // gets the parent of our node in the target array
			targetParentY = getParent(targetX1, targetY1)[1];

			originX1 = originParentX; // set our new origin value as its origin parent value
			originY1 = originParentY;

			targetX1 = targetParentX; // set our new target value as its parent target value
			targetY1 = targetParentY;
			i5++; // increment i5
		}

		int i35 = 0;
		int i25 = targetArray.length - 1;

		boolean commonPositionOrigin = false;
		boolean commonPositionTarget = false;

		while (commonPositionOrigin == false) // This is where we construct our traverse array out of two arrays which
												// are our origin and target arrays
		{
			if (i35 < originArray.length) // how this essentially works is that we add elements starting from the origin
											// coordinate to the common coordinate to our final array
				traverseArray = addArrayToArray(traverseArray, originArray[i35]);
			if (originArray[i35][0] == commonX && originArray[i35][1] == commonY)
				commonPositionOrigin = true;
			i35++;
		}
		while (i25 >= 0) // then once we reach our common coordinate in the origin array we then start
							// from the common coordinate in our target array
		{

			if (commonPositionTarget == true)
				traverseArray = addArrayToArray(traverseArray, targetArray[i25]); // we then work backwards to the first
																					// element in our target array which
																					// is the last node we want to reach
			if (targetArray[i25][0] == commonX && targetArray[i25][1] == commonY)
				commonPositionTarget = true;

			i25--;
		}
		// System.out.println(commonX);
		// System.out.println(commonY);
		// System.out.println(Arrays.deepToString(originArray));
		// System.out.println(Arrays.deepToString(targetArray));
		// System.out.println(Arrays.deepToString(traverseArray));

		return traverseArray; // return our 2D array which contains the path in order of which nodes the robot
								// will come across first
	}

	public int[][] losslessCompressionAlgorithm(int[][] traverseArray, int x1, int y1) // this is a lossless compression
																						// algorithm that reduces the
																						// number of elements in our
																						// traverse array so that our
																						// final path is both optimal
																						// and takes up less memory
	{ // it compresses the 2D array by identifying consecutive sequences of multiple X
		// or Y values all appearing in a row and coalescing them into one large step
		int[][] compressionArray = traverseArray; // we declare a new array to briefly store our old array
		traverseArray = new int[0][2]; // declare a new traverse array
		int skipToNext = 0; // declares the next value for our i0 to skip to
		for (int i0 = 0; i0 < compressionArray.length; i0++) // for the entire length of the array do the following:
		{
			boolean XMode = true; // decide whether or not we're looking to compress a set of consecutive X
									// coordinates or a set of consecutive Y coordinates
			boolean YMode = true;

			for (int i = i0 + 1; i < compressionArray.length; i++) // we compare every element to every other element in
																	// our traverse array, we can make this a bit more
																	// efficient by implementing a skip variable for our
																	// i0 counter to be set to
			{
				if (compressionArray[i0][0] == compressionArray[i][0] && XMode == true) // if our X coordinate is the
																						// same for multiple steps in a
																						// row and we are currently
																						// compressing a series of
																						// repeated X coordinates as
																						// denoted by "xMode" then we
																						// assign the
				{ // skipToNext variable as the counter i which represents the position at which
					// our last repeated X coordinate can be found
					skipToNext = i; // set our skipToNext varaible as our value "i"
					YMode = false; // set the alternate variable equal to false to prohibit it from running
				} else if (compressionArray[i0][1] == compressionArray[i][1] && YMode == true) // same as above but for
																								// consecutive Y
																								// coordinates in a row
				{
					skipToNext = i;
					XMode = false;
				} else // if the sequence of consecutive X or Y coordinates ends then what we can do is
						// identify this spot as the point at which we can skip to the next position i0
						// and add the last (x, y) coordinate to the sequence
				{
					traverseArray = addArrayToArray(traverseArray, compressionArray[i - 1]); // adds this final
																								// consecutive
																								// coordinate to our
																								// compressed array
					i0 = skipToNext - 1;
					break;
				}
			}
		}
		traverseArray = addArrayToArray(traverseArray, compressionArray[compressionArray.length - 1]);
		int[] targetCoord = new int[2]; // this adds the coordinate of the final node (in this case the target) to the
										// compressed array
		targetCoord[0] = x1;
		targetCoord[1] = y1;
		traverseArray = addArrayToArray(traverseArray, targetCoord); // adds target coordinate to our array
		return traverseArray; // returns our "finalPath" array
	}
}

class PriorityNode // This class is defined by the fact that it's used to store the priority node
					// objects and has its own priority queue which is stored in the form of an
					// array
{
	PriorityNode(int x, int y, double nodeCost) // declares our priority node object which stores its (x, y) coordinate
												// and it's node cost
	{
		this.x = x;
		this.y = y;
		this.nodeCost = nodeCost;
	}

	private int x;
	private int y;
	private double nodeCost;
	private PriorityNode[] priorityQueue; // declares the priority queue array for our class

	public int getY() // some getter functions to extract information about our priority nodes
	{
		return y;
	}

	public int getX() {
		return x;
	}

	public double getNodeCost() {
		return nodeCost;
	}

	public PriorityNode[] getPriorityQueue() // returns our priority queue so we can extract the next priority node
	{
		return priorityQueue;
	}

	public void resetQueueArray() // method to reset the priority queue on new runs of new mazes
	{
		priorityQueue = new PriorityNode[0];
	}

	public void addPriorityNode(PriorityNode priorityNode) // this method is used to queue up priority nodes based on
															// their node cost
	{ // I realise that there is already a priority queue library with a priority
		// queue object in java but I wanted the queue to store extra information (i.e I
		// wanted a collection of priority node objects as opposed to a bunch of ordered
		// nodeCost doubles)
		int arrayLength = (priorityQueue.length);
		PriorityNode[] newArray = new PriorityNode[arrayLength + 1]; // declare a new array that is one size larger than
																		// the old one
		int positionCounter = 0;

		boolean duplicatePNodeFound = false; // to avoid adding the same node twice we have to do a check first
		int countNode = arrayLength - 1;

		while (countNode >= 0 && duplicatePNodeFound == false) // we compare each (x, y) coordinate already in our
																// priority queue to the nodes (x, y) coordinate we want
																// to add
		{
			if (priorityNode.getX() == priorityQueue[countNode].getX() && // if we find a duplicate node being added to
																			// our queue then we just don't add the node
					priorityNode.getY() == priorityQueue[countNode].getY())
				duplicatePNodeFound = true;
			countNode--;

		}

		if (duplicatePNodeFound == false) // as long as we don't find a duped node then we can add it
		{
			for (int countMaze1 = 0; countMaze1 < arrayLength; countMaze1++) // firstly we compare the nodes node cost
																				// value to the other nodecost values in
																				// our queue
			{
				if (priorityNode.getNodeCost() > priorityQueue[countMaze1].getNodeCost()) {
					positionCounter++; // every time we find a node cost value that the node we are adding is greater
										// than then we increment the position counter
				}
			}

			if (positionCounter > 0) // as long as the position counter is not the position 0 in our queue then we
										// add the nodes in front of the node first
			{
				for (int countMaze2 = 0; countMaze2 < positionCounter; ++countMaze2) {
					newArray[countMaze2] = priorityQueue[countMaze2];
				}
			}

			newArray[positionCounter] = priorityNode; // the position of the node we are adding is denoted by the value
														// of the position counter we calculated above (we insert the
														// priority node in the position)
			for (int countMaze3 = positionCounter; countMaze3 < arrayLength; countMaze3++) // for every element that our
																							// added node has a lower
																							// nodecost value than the
																							// ones in our queue then we
																							// add these values last
			{
				newArray[countMaze3 + 1] = priorityQueue[countMaze3];
			}
			priorityQueue = newArray; // our priority queue is now set to the new array we created
		}
		// else System.out.print("Dupe found");
	}

	public void removeTraversedNode() // this method removes the last element we added to our priority queue when we
										// expand the node
	{
		int arrayLength = (priorityQueue.length);
		PriorityNode[] newArray = new PriorityNode[arrayLength - 1]; // we make our new array a size less than our old
																		// one

		for (int countMaze2 = 1; countMaze2 < arrayLength; countMaze2++) // we then duplicate our array to a temporary
																			// array and every node in our priority
																			// queue is moved up a spot
		{
			newArray[countMaze2 - 1] = priorityQueue[countMaze2];
		}
		priorityQueue = newArray; // set our priority queue array as our temporary array
	}
}
