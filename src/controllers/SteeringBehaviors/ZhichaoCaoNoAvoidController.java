/**
 * This is a controller for the PTSP that extends greedyController makes use of steering behaviors.
 * CS680 Game-AI
 * Zhichao Cao
 * zc77@drexel.edu
 */
package controllers.SteeringBehaviors;

import framework.core.*;
import framework.graph.Graph;
import framework.graph.Node;
import framework.graph.Path;
import framework.utils.Vector2d;

import java.awt.*;
import java.util.HashMap;

public class ZhichaoCaoNoAvoidController extends Controller
{
	/**
	 * Graph for this controller.
	 */
	private Graph m_graph;

	/**
	 * Node in the graph, closest to the ship position.
	 */
	private Node m_shipNode;

	/**
	 * Path to the closest waypoint in the map
	 */
	private Path m_pathToClosest;

	/**
	 * Hash map that matches waypoints in the map with their closest node in the
	 * graph.
	 */
	private HashMap<Waypoint, Node> m_collectNodes;

	/**
	 * Closest waypoint to the ship.
	 */
	private Waypoint m_closestWaypoint;

	/**
	 * Constructor, that receives a copy of the game state
	 * 
	 * @param a_gameCopy
	 *            a copy of the game state
	 * @param a_timeDue
	 *            The time the initialization is due. Finishing this method
	 *            after a_timeDue will disqualify this controller.
	 */
	public ZhichaoCaoNoAvoidController(Game a_gameCopy, long a_timeDue) {
		// Init the graph.
		m_graph = new Graph(a_gameCopy);

		// Init the structure that stores the nodes closest to all waypoitns
		m_collectNodes = new HashMap<Waypoint, Node>();
		for (Waypoint way : a_gameCopy.getWaypoints()) {
			m_collectNodes.put(way,	m_graph.getClosestNodeTo(way.s.x, way.s.y, true));
		}

		// Calculate the closest waypoint to the ship.
		calculateClosestWaypoint(a_gameCopy);
	}

	/**
	 * This function is called every execution step to get the action to
	 * execute.
	 * 
	 * @param a_gameCopy
	 *            Copy of the current game state.
	 * @param a_timeDue
	 *            The time the next move is due
	 * @return the integer identifier of the action to execute (see interface
	 *         framework.core.Controller for definitions)
	 */
	public int getAction(Game a_gameCopy, long a_timeDue) {
		// Get the path to the closest node, if my ship moved.
		Node oldShipId = m_shipNode;
		m_shipNode = m_graph.getClosestNodeTo(a_gameCopy.getShip().s.x, a_gameCopy.getShip().s.y, true);
		if (oldShipId != m_shipNode || m_pathToClosest == null) {
			// Calculate the closest waypoint to the ship.
			calculateClosestWaypoint(a_gameCopy);

			if (m_shipNode == null) {
				// No node close enough and collision free. Just go for the closest.
				m_shipNode = m_graph.getClosestNodeTo(a_gameCopy.getShip().s.x,	a_gameCopy.getShip().s.y, false);
			}

			// And get the path to it from my location.
			m_pathToClosest = m_graph.getPath(m_shipNode.id(), m_collectNodes.get(m_closestWaypoint).id());
		}
		
		// Wanna know the distance to the closest obstacle ahead of the ship? Try:
		// double distanceToColl = a_gameCopy.getMap().distanceToCollision(a_gameCopy.getShip().s, a_gameCopy.getShip().d, 1000);
		// System.out.println("DIST: " + distanceToColl);

		// We treat this differently if we can see the waypoint:
		boolean isThereLineOfSight = a_gameCopy.getMap().LineOfSight(a_gameCopy.getShip().s, m_closestWaypoint.s);
		if (isThereLineOfSight) {
			return motorControl(a_gameCopy);
		}

		// The waypoint is behind an obstacle, select which is the best action
		// to take.
		double minDistance = Float.MAX_VALUE;
		int bestAction = -1;
		double bestDot = -2;

		if (m_pathToClosest != null) // We should have a path...
		{
			int startAction = Controller.ACTION_NO_FRONT;
			// For each possible action...
			for (int action = startAction; action < Controller.NUM_ACTIONS; ++action) {
				// Simulate that we execute the action and get my potential
				// position and direction
				Game forThisAction = a_gameCopy.getCopy();
				forThisAction.getShip().update(action);
				Vector2d nextPosition = forThisAction.getShip().s;
				Vector2d potentialDirection = forThisAction.getShip().d;

				// Get the next node to go to, from the path to the closest waypoint
				Node nextNode = getNextNode();
				Vector2d nextNodeV = new Vector2d(nextNode.x(), nextNode.y());
				nextNodeV.subtract(nextPosition);
				nextNodeV.normalise(); // This is a unit vector from my position pointing towards the next node to go to.
				double dot = potentialDirection.dot(nextNodeV); // Dot product between this vector and where the ship is facing to.

				// Get the distance to the next node in the tree and update the total distance until the closest waypoint
				double dist = nextNode.euclideanDistanceTo(nextPosition.x, nextPosition.y);
				double totalDistance = m_pathToClosest.m_cost + dist;

				// System.out.format("Action: %d, total distance: %.3f, distance to node: %.3f, dot: %.3f\n",action, totalDistance, dist, dot);

				// Keep the best action so far.
				if (totalDistance < minDistance) {
					minDistance = totalDistance;
					bestAction = action;
					bestDot = dot;
				}
				// If the distance is the same, keep the action that faces the
				// ship more towards the next node
				else if ((int) totalDistance == (int) minDistance && dot > bestDot) {
					minDistance = totalDistance;
					bestAction = action;
					bestDot = dot;
				}
			}

			// This is the best action to take.
			return bestAction;			
		}

		// Default (something went wrong).
		return Controller.ACTION_NO_FRONT;
	}

	/**
	 * Returns the first node in the way to the destination
	 * 
	 * @return the node in the way to destination.
	 */
	private Node getNextNode() {
		Node n0 = m_graph.getNode(m_pathToClosest.m_points.get(0));

		// If only one node in the path, return it.
		if (m_pathToClosest.m_points.size() == 1)
			return n0;

		// Heuristic: Otherwise, take the closest one to the destination
		Node n1 = m_graph.getNode(m_pathToClosest.m_points.get(1));
		Node destination = m_graph.getNode(m_pathToClosest.m_destinationID);

		if (n0.euclideanDistanceTo(destination) < n1.euclideanDistanceTo(destination))
			return n0;
		else
			return n1;
	}

	/**
	 * Calculates the closest waypoint to the ship.
	 * 
	 * @param a_gameCopy
	 *            the game copy.
	 */
	private void calculateClosestWaypoint(Game a_gameCopy) {
		double minDistance = Double.MAX_VALUE;
		for (Waypoint way : a_gameCopy.getWaypoints()) {
			if (!way.isCollected()) // Only consider those not collected yet.
			{
				double fx = way.s.x - a_gameCopy.getShip().s.x, fy = way.s.y - a_gameCopy.getShip().s.y;
				double dist = Math.sqrt(fx * fx + fy * fy);
				if (dist < minDistance) {
					// Keep the minimum distance.
					minDistance = dist;
					m_closestWaypoint = way;
				}
			}
		}
	}

	/**
	 * This is a debug function that can be used to paint on the screen.
	 * 
	 * @param a_gr
	 *            Graphics device to paint.
	 */
	public void paint(Graphics2D a_gr) {
		// m_graph.draw(a_gr);
		a_gr.setColor(Color.yellow);
		Path pathToClosest = getPathToClosest();
		if (pathToClosest != null)
			for (int i = 0; i < pathToClosest.m_points.size() - 1; ++i) {
				Node thisNode = m_graph.getNode(pathToClosest.m_points.get(i));
				Node nextNode = m_graph.getNode(pathToClosest.m_points.get(i + 1));
				a_gr.drawLine(thisNode.x(), thisNode.y(), nextNode.x(), nextNode.y());
			}
	}

	/**
	 * Returns the path to the closest waypoint. (for debugging purposes)
	 * 
	 * @return the path to the closest waypoint
	 */
	public Path getPathToClosest() {
		return m_pathToClosest;
	}

	/**
	 * Returns the graph. (for debugging purposes)
	 * 
	 * @return the graph.
	 */
	public Graph getGraph() {
		return m_graph;
	}
    
    private int motorControl(Game a_gameCopy) {
//    	//Implementation of Steering Behaviors
    	int bestAction = Controller.ACTION_NO_FRONT;
    	int thrust = 0;
    	int turn = 0;
    	Vector2d curDir = a_gameCopy.getShip().d; 	//Current direction
    	Vector2d curSpd = a_gameCopy.getShip().v;	//Current speed
    	Vector2d curPos = a_gameCopy.getShip().s;	//Current position of ship
    	Vector2d targetPoint = m_closestWaypoint.s.copy();
    	targetPoint.subtract(curPos);
    	double distance = targetPoint.mag();	//D
    	targetPoint.normalise();	//ND
    	
    	double maxspeed = 0.4;	//Default standard for speed
		if (distance > 100) {
			//Point is far away to the target point, so we allow ship to accelerate to the 0.8
			maxspeed = 0.8;
		}
		else if (distance < 30) {
			//Point is close to the target point, so we need to control it not to accelerate too much for accurate movement
			maxspeed = 0.25;
		}
		
		double alpha = getAlpha(curDir, targetPoint);	//Angle between current direction and target point
		double veloAlpha = 0;
		double curSpdCopy = curSpd.mag();
		
		//Choice on turnning
		if(curSpdCopy > 0) {
			//The ship is moving, calculate the angle between its direction and its actual velocity
			curSpd.normalise();
			veloAlpha = getAlpha(curSpd, curDir);	//Angle between current velocity and current direction
		}
		if(targetPoint.dot(curDir) > 0.999) {
			//If ship is almost facing the target point, we do not need turn
			turn = 0;
		}
		else if(alpha > 0 && alpha <= Math.PI) {
			//If the target point is at left side, we turn left
			turn = 1;
		}
		else if(alpha < 0 && alpha >= -Math.PI) {
			//If the target point is at right side, we turn right
			turn = -1;
		}
		else if(alpha < -Math.PI) {
			//Similar situation to turn left
			turn = 1;
		}
		else if(alpha > Math.PI) {
			//Similar situation to turn right
			turn = -1;
		}
		
		//Choice on acceleration
		if ( veloAlpha == Math.PI/2	|| veloAlpha == -Math.PI/2) {
			//If the angle between direction and actual velocity is an right angle, we do not accelerate 
			thrust = 0;
		}
		else if(curSpdCopy > maxspeed) {
			//If the current speed is larger than the max speed we defined in different situation before,
			//we do not accelerate
			thrust = 0;
		}
		else {
			//In other situations, we accelerate
			thrust = 1;
		}
		
		if(thrust == 0 && turn == 0) {
			bestAction = Controller.ACTION_NO_FRONT;
    	}
    	else if(thrust == 0 && turn == 1) {
    		bestAction = Controller.ACTION_NO_LEFT;
    	}
    	else if(thrust == 0 && turn == -1) {
    		bestAction = Controller.ACTION_NO_RIGHT;
    	}
    	else if(thrust == 1 && turn == 0) {
    		bestAction = Controller.ACTION_THR_FRONT;
    	}
    	else if(thrust == 1 && turn == 1) {
    		bestAction = Controller.ACTION_THR_LEFT;
    	}
    	else if(thrust == 1 && turn == -1) {
    		bestAction = Controller.ACTION_THR_RIGHT;
    	}
    	
    	return bestAction;
    	
    }
    
    private double getAlpha(Vector2d v1, Vector2d v2) {
    	//Calculate angle between v1 and v2
    	double alpha = calRealAngle(v1) - calRealAngle(v2);
    	return alpha;
    }
    
    private double calRealAngle(Vector2d vec) {
    	//Calculate the real angle
    	double value = 0;
    	if(vec.y > 0) {
    		value = Math.acos(vec.x);
    	}
    	else if(vec.y < 0) {
    		value = -Math.acos(vec.x);
    	}
    	else if(vec.x > 0) {
    		value = 0;
    	}
    	else {
    		value = -Math.PI;
    	}
    	return value;
    }
}
