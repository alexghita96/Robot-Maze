import uk.ac.warwick.dcs.maze.logic.*;
import java.util.*;
import java.awt.event.*;
import java.awt.Color;
import java.awt.Font;
import java.awt.Point;
import javax.swing.*;

// Custom comparator for the A* priority queue; will order according to Manhattan distance to target
class ManhattanDistanceComparator implements Comparator<SquareData> {
    @Override
    public int compare(SquareData a, SquareData b) {
        if (a.getPriority() < b.getPriority()) {
            return -1;
        }
        if (a.getPriority() > b.getPriority()) {
            return 1;
        }
        return 0;
    }
}

// Class for storing a square's coordinates and the square's priority
class SquareData {
	private Point location;
	private int priority;

	public SquareData(Point location, int priority) {
		this.location = location;
		this.priority = priority;
	}

	public Point getLocation() {
		return location;
	}

	public int getPriority() {
		return priority;
	}

	// Print the square data in a readable format; useful for debugging
	public void printData() {
		System.out.println("X = " + location.x + ", Y = " + location.y + ", Priority = " + priority);
	}
}

// Store data about a single junction - its coordinates and the direction the robot arrived from
class JunctionRecorder {
	private Point junctionSquare;
	private int arrived;

	public JunctionRecorder(Point junctionSquare, int arrived) {
		this.junctionSquare = junctionSquare;
		this.arrived = arrived;
	}

	public Point getJunctionSquare() {
		return junctionSquare;
	}

	public int getArrived() {
		return arrived;
	}

	// Print the junction in a readable format; useful for debugging
	public void printJunction() {
		String direction;

		switch (arrived) {
			case (IRobot.NORTH):
				direction = "NORTH";
				break;
			case (IRobot.EAST):
				direction = "EAST";
				break;
			case (IRobot.SOUTH):
				direction = "SOUTH";
				break;
			default:
				direction = "WEST";
				break;
		}

		System.out.println("Junction at (x=" + junctionSquare.x + ",y=" + junctionSquare.y + "), heading from " + direction);
	}
}

// Use DFS to explore the entire maze and store this information into an array, then use A* to find the shortest path
public class GrandFinale {
	private final int MAX_MAZE_SIZE = 405; // Constant for maximum maze size

	private int pollRun; // Incremented after each pass
	private int stepCounter; // Store the index of the step in the shortestPath sequence that the robot needs to follow
	private int[] shortestPath; // Store the sequence of headings the robot has to use to get to the target on the shortest path
	private int[][] mazeGrid; /* Store information about the grid. -1 will always represent a wall.
								 In the first run, a cell will be 1 if its corresponding square is a visited junction,
								 or 0 otherwise; in subsequent runs, non-negative numbers will represent the
								 minimum number of steps required to get to that square */
	private byte robotSearchMode; /* Store the robot's search mode:
									 -> 0 for Explore Mode;
									 -> 1 for Backtrack Mode;
									 -> 2 for Shortest Path Mode;
									 -> 3 for Prim Maze Game Mode;
									 -> 4 for Loopy Maze Game Mode;
									 -> 5 for Blank Maze Game Mode; */
	private Point startingSquare; // Store the coordinates for the starting square
	private Stack<JunctionRecorder> junctionStack; // Store only the junctions the robot has been through and to which it can currently backtrack to
	private PriorityQueue<SquareData> searchSquares; /* Store the coordinates of the point from which the A* algorithm will expand its search
										        		and their priority based on the Manhattan distance to the target
										     		 */
	// Game stuff
	private int mazeX, mazeY; // Size of the current maze
	private int tetriminoIndex; // The current tetrimino's index in the list -- Blank Maze Game Mode
	private int targetHeading; // The target's current heading -- Prim & Loopy Maze Game Mode
	private int passageCounter; // Count how many passage squares are left unexplored -- Loopy Maze Game Mode
	private int squaresLeft; // Squares left to visit -- Loopy Maze Game Mode
	private int[][] mazeGridFrame; // Store only -1 for walls and 0 otherwise; useful for reintialising the maze grid
	private boolean mazeIsLoopy; // True if the maze is loopy, false otherwise; needed to determine which kind of maze the robot is running on
	private boolean needTetrimino; // True if a new tetrimino must be generated, false otherwise -- Blank Maze Game Mode
	private boolean tetriminoLanded; // True if the current tetrimino just landed, false otherwise -- Blank Maze Game Mode
	private boolean[][] seenSquare; // True if the square has been visited, false otherwise -- Loopy Maze Game Mode
	private Point originalStartingSquare; // The original starting square for this maze -- Prim & Loopy Maze Game Mode
	private Point originalTargetSquare; // The original target square for this maze -- Prim & Loopy Maze Game Mode
	private Point currentRobotSquare; // The current robot square -- Prim & Loopy Maze Game Mode
	private Point currentTargetSquare; // The current target square -- Prim & Loopy Maze Game Mode
	private Point[] tetrimino; // Hold the coordinates of each tetrimino square -- Blank Maze Game Mode
	private Point[][] tetriminoList; // Hold the list of all possible tetriminos -- Blank Maze Game Mode
	private InputPanel userInputPanel = new InputPanel(); // Input panel that will appear when in game mode to read user input
	private Maze theMaze; // The maze object; useful when calling the reset method

	/*
		Utility methods
	*/

	// Randomly choose a heading which doesn't lead into a wall from a set of given headings
	private int randomHeading(IRobot robot, int[] headingSet) {
		int randomNumber;

		// While the selected heading leads into a wall, pick another one randomly
		do {
			randomNumber = (int)(Math.random() * headingSet.length);
		} while (headingTowardsSquareType(robot, headingSet[randomNumber], IRobot.WALL) == true || testCoordinates(robot.getLocation(), headingSet[randomNumber]).equals(robot.getTargetLocation()) == true);

		return headingSet[randomNumber];
	}

	// Return the coordinates of a square adjacent to a specific square, given a heading
	private Point testCoordinates(Point currentSquare, int heading) {
		switch (heading) {
			case IRobot.NORTH:
				return new Point(currentSquare.x, currentSquare.y - 1);
			case IRobot.EAST:
				return new Point(currentSquare.x + 1, currentSquare.y);
			case IRobot.SOUTH:
				return new Point(currentSquare.x, currentSquare.y + 1);
			default:
				return new Point(currentSquare.x - 1, currentSquare.y);
		}
	}

	// Test if the selected heading leads to a squareType square
	private boolean headingTowardsSquareType(IRobot robot, int heading, int squareType) {
		robot.setHeading(heading);

		if (robot.look(IRobot.AHEAD) == squareType && testCoordinates(robot.getLocation(), heading).equals(robot.getTargetLocation()) == false) {
			return true;
		}
		return false;
	}

	// Return the number of squareType squares adjacent to the square currently occupied by the robot
	private int countAdjacentSquares(IRobot robot, int squareType) {
		int squareTypePaths = 0;
		int initialHeading = robot.getHeading();

		// Look around the robot and count the PASSAGE squares
		for (int heading = IRobot.NORTH; heading <= IRobot.WEST; heading++) {
			robot.setHeading(heading);
			if (robot.look(IRobot.AHEAD) == squareType && testCoordinates(robot.getLocation(), heading).equals(robot.getTargetLocation()) == false) {
				squareTypePaths++;
			}
		}
		robot.setHeading(initialHeading);

		return squareTypePaths;
	}

	// Return the opposite heading for the provided heading
	private int reverseHeading(int heading) {
		return (heading + 2) % 4 + IRobot.NORTH;
	}

	// Return the Manhattan distance between two squares
	private int manhattanDistance(Point startSquare, Point targetSquare) {
		return Math.abs(startSquare.x - targetSquare.x) + Math.abs(startSquare.y - targetSquare.y);
	}

	// Copies the information from the source array to the destination array
	private void copyArray(int[][] sourceArray, int[][] destinationArray) {
		for (int i = 0; i < MAX_MAZE_SIZE; i++) {
			for (int j = 0; j < MAX_MAZE_SIZE; j++) {
				destinationArray[i][j] = sourceArray[i][j];
			}
		}
	}

	/*
		Movement decision methods
	*/

	// Decide what to do when the robot is at a dead end
	private int deadEndMove(IRobot robot) {
		int heading;

		// Look for the only non-WALL exit
		for (heading = IRobot.NORTH; heading <= IRobot.WEST; heading++) {
			if (headingTowardsSquareType(robot, heading, IRobot.WALL) == false) {
				break;
			}
		}
		return heading;
	}

	// Decide what to do when the robot is travelling down a corridor
	private int corridorMove(IRobot robot) {
		int heading;
		int oppositeHeading = reverseHeading(robot.getHeading());

		// Look for a non-WALL exit which is not the same as the one the robot got here from (oppositeHeading)
		for (heading = IRobot.NORTH; heading <= IRobot.WEST; heading++) {
			if (heading == oppositeHeading) {
				continue;
			}
			if (headingTowardsSquareType(robot, heading, IRobot.WALL) == false) {
				break;
			}
		}
		return heading;
	}

	/*
		Maze exploration methods
	*/

	// Explore new squares of the maze
	private int exploreControl(IRobot robot) {
		int heading;
		int passagePaths;
		int initialHeading;
		int headingCounter;
		int[] headingSet;

		passagePaths = countAdjacentSquares(robot, IRobot.PASSAGE);
		initialHeading = robot.getHeading();

		// If there are no PASSAGE exits around the robot, switch to backtrack mode
		if (passagePaths == 0) {
			robotSearchMode = 1;
			heading = backtrackControl(robot);
		}
		// Otherwise, pick a random PASSAGE exit for the robot to head to
		else {
			headingSet = new int[passagePaths];
			headingCounter = -1;
			for (heading = IRobot.NORTH; heading <= IRobot.WEST; heading++) {
				if (headingTowardsSquareType(robot, heading, IRobot.PASSAGE) == true) {
					headingSet[++headingCounter] = heading;
				}
			}
			heading = randomHeading(robot, headingSet);
		}

		return heading;
	}

	// Backtrack to a previously encountered junction
	private int backtrackControl(IRobot robot) {
		int nonWallExits;
		int heading;

		nonWallExits = 4 - countAdjacentSquares(robot, IRobot.WALL);
		// Treat the starting square as a junction
		if (robot.getLocation().equals(startingSquare)) {
			nonWallExits = 3;
		}

		switch (nonWallExits) {
			case 1:
				heading = deadEndMove(robot);
				break;
			case 2:
				heading = corridorMove(robot);
				break;
			default:
				// If the robot got back into an explored junction that isn't the last explored junction, turn back
				if (junctionStack.empty() == false && mazeGrid[robot.getLocation().x][robot.getLocation().y] == 1 &&
					robot.getLocation().equals(junctionStack.peek().getJunctionSquare()) == false) {
					heading = reverseHeading(robot.getHeading());
					mazeIsLoopy = true;
				}
				// If there are unexplored exits, enter in explorer mode and randomly choose one
				else if (countAdjacentSquares(robot, IRobot.PASSAGE) > 0) {
					robotSearchMode = 0;
					heading = exploreControl(robot);
				}
				// Otherwise, go back to a previous junction if the stack isn't empty
				else {
					// Reverse heading
					heading = junctionStack.peek().getArrived();
					heading = reverseHeading(heading);
					junctionStack.pop();
					// If the stack just emptied, then it means the robot got back to the starting area, so it will switch to Shortest Path Mode
					if (junctionStack.empty() == true) {
						robotSearchMode = 2;
						prepareMazeGrid(startingSquare);
						mazeGridFrame = new int[MAX_MAZE_SIZE][MAX_MAZE_SIZE]; // This will be used in Game mode
						copyArray(mazeGrid, mazeGridFrame);
						// Use the A* algorithm to find the shortest path to the target and store it
						aStarAlgorithm(robot.getLocation(), robot.getTargetLocation());
						retracePath(robot.getLocation(), robot.getTargetLocation());
						heading = shortestPath[1];
						stepCounter = 2;
					}
				}
				break;
		}

		return heading;
	}

	/*
		Shortest Path methods
	*/

	// Erase all the 1's from the grid and set the starting square to 1
	private void prepareMazeGrid(Point startSquare) {
		for (int i = 0; i < mazeGrid.length; i++) {
			for (int j = 0; j < mazeGrid[i].length; j++) {
				if (mazeGrid[i][j] > 0) {
					mazeGrid[i][j] = 0;
				}
			}
		}

		mazeGrid[startSquare.x][startSquare.y] = 1;
	}

	// Use the A* Algorithm to find the shortest path between two points
	private void aStarAlgorithm(Point startSquare, Point targetSquare) {
		SquareData currentSquare;
		int priority;
		Point testSquare;

		Comparator<SquareData> comparator = new ManhattanDistanceComparator();
		searchSquares = new PriorityQueue<SquareData>(4 * MAX_MAZE_SIZE, comparator);
		searchSquares.add(new SquareData(startSquare, 0)); // Add the starting location to the priority queue.

		// While the priority queue isn't empty, take an square from it and expand on the surrounding squares
		while (searchSquares.peek() != null) {
			currentSquare = searchSquares.remove();
			if (currentSquare.getLocation().equals(targetSquare)) {
				break;
			}

			// Look for unvisited squares around the current square
			for (int heading = IRobot.NORTH; heading <= IRobot.WEST; heading++) {
				testSquare = testCoordinates(currentSquare.getLocation(), heading);
				// If the test square is out of bounds or a wall, skip it
				if (testSquare.x < 0 || testSquare.x > 400 || testSquare.y < 0 || testSquare.y > 400 || mazeGrid[testSquare.x][testSquare.y] == -1) {
					continue;
				}
				// If the selected square is an unvisited square or the number of steps stored in that square isn't minimum, update it
				if (mazeGrid[testSquare.x][testSquare.y] == 0 || mazeGrid[testSquare.x][testSquare.y] > mazeGrid[currentSquare.getLocation().x][currentSquare.getLocation().y] + 1) {
					mazeGrid[testSquare.x][testSquare.y] = mazeGrid[currentSquare.getLocation().x][currentSquare.getLocation().y] + 1;
					priority = mazeGrid[testSquare.x][testSquare.y] + manhattanDistance(testSquare, targetSquare);
					searchSquares.add(new SquareData(testSquare, priority));
				}
			}
		}
	}

	// Get the sequence of steps required to to reach the target using the shortest path
	private void retracePath(Point startSquare, Point targetSquare) {
		Point currentPoint;
		Point testSquare;

		shortestPath = new int[MAX_MAZE_SIZE * MAX_MAZE_SIZE];
		currentPoint = targetSquare;
		stepCounter = mazeGrid[currentPoint.x][currentPoint.y] - 1;

		/* Start from the target square and look for squares that have been reached with one less step than the number of steps needed to reach the current
		   square (this means the robot got to the current square from that square). Repeat until the starting square is reached.
		 */
		while (currentPoint.equals(startSquare) == false) {
			for (int heading = IRobot.NORTH; heading <= IRobot.WEST; heading++) {
				testSquare = testCoordinates(currentPoint, heading);
				if (testSquare.x < 1 || testSquare.x > 400 || testSquare.y < 1 || testSquare.y > 400) {
					continue;
				}
				if (mazeGrid[testSquare.x][testSquare.y] == mazeGrid[currentPoint.x][currentPoint.y] - 1) {
					shortestPath[stepCounter--] = reverseHeading(heading);
					currentPoint = testSquare;
					break;
				}
			}
		}
	}

	/*
		Game mode methods
	*/

	// Simple game where the target chases you on Prim mazes
	private int primMazeGame(IRobot robot, int broadcastSignal) {
		int robotHeading;

		// Reinitialise the maze grid and update the robot's square
		copyArray(mazeGridFrame, mazeGrid);
		currentRobotSquare = robot.getLocation();
		// Compute the shortest path from the target to the robot
		prepareMazeGrid(currentTargetSquare);
		aStarAlgorithm(currentTargetSquare, currentRobotSquare);
		retracePath(currentTargetSquare, currentRobotSquare);
		// Set the headings for the target and the robot
		targetHeading = shortestPath[1];
		robotHeading = userInputPanel.getInputHeading();
		// If the robot is about to bump into the target, the target will not move
		if (testCoordinates(currentRobotSquare, robotHeading).equals(currentTargetSquare) == false) {
			currentTargetSquare = testCoordinates(currentTargetSquare, targetHeading);
		}
		// Set the new locations and update the maze;
		robot.getMaze().setFinish(currentTargetSquare.x, currentTargetSquare.y);
		robot.getMaze().setStart(currentRobotSquare.x, currentRobotSquare.y);
		EventBus.broadcast(new Event(broadcastSignal, robot.getMaze()));

		return robotHeading;
	}

	// Simple Pac-Man-like game where you have to visit all squares in loopy mazes before the target catches you
	private int loopyMazeGame(IRobot robot) {
		// If the robot is on a previously unseen square, decrement the number of squares left to visit
		if (seenSquare[robot.getLocation().x][robot.getLocation().y] == false) {
			seenSquare[robot.getLocation().x][robot.getLocation().y] = true;
			squaresLeft--;
		}
		// Check if the player won
		if (squaresLeft == 0) {
			JOptionPane.showMessageDialog(null, "Congratulations! You've won the game! Now see how long you can run from the target.");
		}

		// The rest is the same as in the Prim Maze Game Mode, except for the broadcast signal, so call that method
		return primMazeGame(robot, 102);
	}

	// Tetris game for blank mazes
	private void blankMazeGame(IRobot robot) {
		// If a tetrimino needs to be generated, select one at random and update the grid
		if (needTetrimino == true) {
			needTetrimino = false;
			tetriminoLanded = false;
			selectNewTetrimino(robot);
			for (int i = 0; i <= 3; i++) {
				robot.getMaze().setCellType(tetrimino[i].x, tetrimino[i].y, 2);
			}
		}
		// Otherwise, move the piece
		else {
			playerMoveTetrimino(robot);
			moveTetrimino(robot, 0, 1);
			// If it landed, check if there are any lines to erase and if the game is over
			if (tetriminoLanded(robot) == true) {
				eraseLines(robot);
				if (gameOver(robot) == true) {
					JOptionPane.showMessageDialog(null, "Game over.");
					resetMaze(robot);
				}
				else {
					needTetrimino = true;
				}
			}
		}
	}

	// Choose a new tetrimino at random
	private void selectNewTetrimino(IRobot robot) {
		int randomNumber;

		randomNumber = (int)(Math.random() * 28);

		for (int i = 0; i <= 3; i++) {
			tetrimino[i].x = tetriminoList[randomNumber][i].x;
			tetrimino[i].y = tetriminoList[randomNumber][i].y;
		}
		tetriminoIndex = randomNumber;
	}

	// Move the tetrimino in the specified direction
	private void moveTetrimino(IRobot robot, int moveX, int moveY) {
		boolean keepGoing = true;

		// First check if it can go in the specified direction
		for (int i = 0; i <= 3; i++) {
			if (robot.getMaze().getCellType(tetrimino[i].x + moveX, tetrimino[i].y + moveY) == 2
				&& partOfTetrimino(robot, new Point(tetrimino[i].x + moveX, tetrimino[i].y + moveY)) == false) {
				keepGoing = false;
				break;
			}
		}

		// If it can go, move it in that direction
		if (keepGoing == true) {
			// Clear the previous location
			for (int i = 0; i <= 3; i++) {
				robot.getMaze().setCellType(tetrimino[i].x, tetrimino[i].y, 1);
			}

			// Fill the new location
			for (int i = 0; i <= 3; i++) {
				tetrimino[i].x += moveX;
				tetrimino[i].y += moveY;
				robot.getMaze().setCellType(tetrimino[i].x, tetrimino[i].y, 2);
			}
		}
	}

	// Move the tetrimino according to the player's input
	private void playerMoveTetrimino(IRobot robot) {
		int userInput;

		userInput = userInputPanel.getTetrisInput();

		if (userInput == 'W' || userInput == 'w') {
			rotateTetrimino(robot);
		}
		else if (userInput == 'A' || userInput == 'a') {
			moveTetrimino(robot, -1, 0);
		}
		else if (userInput == 'S' || userInput == 's') {
			moveTetrimino(robot, 0, 1);
		}
		else if (userInput == 'D' || userInput == 'd') {
			moveTetrimino(robot, 1, 0);
		}
	}

	// Rotate a tetrimino
	private void rotateTetrimino(IRobot robot) {
		Point[] newTetrimino = new Point[4];
		boolean keepGoing = true;

		// Get the next rotation of the current tetrimino
		for (int i = 0; i <= 3; i++) {
			newTetrimino[i] = new Point(tetrimino[i].x + tetriminoList[tetriminoIndex / 4 * 4 + (tetriminoIndex + 1) % 4][i].x - tetriminoList[tetriminoIndex][i].x,
										tetrimino[i].y + tetriminoList[tetriminoIndex / 4 * 4 + (tetriminoIndex + 1) % 4][i].y - tetriminoList[tetriminoIndex][i].y);
		}

		for (int i = 0; i <= 3; i++) {
			if (robot.getMaze().getCellType(newTetrimino[i].x, newTetrimino[i].y) == 2
				&& partOfTetrimino(robot, new Point(newTetrimino[i].x, newTetrimino[i].y)) == false) {
				keepGoing = false;
				break;
			}
		}

		// If it can go, move it in that direction
		if (keepGoing == true) {
			// Clear the previous location
			for (int i = 0; i <= 3; i++) {
				robot.getMaze().setCellType(tetrimino[i].x, tetrimino[i].y, 1);
			}

			// Fill the new location
			for (int i = 0; i <= 3; i++) {
				tetrimino[i].x = newTetrimino[i].x;
				tetrimino[i].y = newTetrimino[i].y;
				robot.getMaze().setCellType(tetrimino[i].x, tetrimino[i].y, 2);
			}
		}
		tetriminoIndex = tetriminoIndex / 4 * 4 + (tetriminoIndex + 1) % 4;
	}

	// Check whether the tetrimino has landed
	private boolean tetriminoLanded(IRobot robot) {
		for (int i = 0; i <= 3; i++) {
			if (robot.getMaze().getCellType(tetrimino[i].x, tetrimino[i].y + 1) == 2
				&& partOfTetrimino(robot, new Point(tetrimino[i].x, tetrimino[i].y + 1)) == false) {
				return true;
			}
		}
		return false;
	}

	// Check if there are any complete lines and erase theme
	private void eraseLines(IRobot robot) {
		boolean erase;

		for (int i = 0; i <= 3; i++) {
			erase = true;
			for (int j = 1; j < mazeX - 1; j++) {
				if (robot.getMaze().getCellType(j, tetrimino[i].y) == 1) {
					erase = false;
					break;
				}
			}
			if (erase == true) {
				for (int y = tetrimino[i].y; y >= 2; y--) {
					for (int x = 1; x < mazeX - 1; x++) {
						robot.getMaze().setCellType(x, y, robot.getMaze().getCellType(x, y - 1));
					}
				}
			}
		}
	}

	// Check if this square is part of the tetrimino that is currently falling
	private boolean partOfTetrimino(IRobot robot, Point square) {
		for (int i = 0; i <= 3; i++) {
			if (tetrimino[i].x == square.x && tetrimino[i].y == square.y) {
				return true;
			}
		}

		return false;
	}

	// Check if the Tetris game is over
	private boolean gameOver(IRobot robot) {
		for (int i = 0; i <= 3; i++) {
			if (tetrimino[i].y == 1) {
				return true;
			}
		}
		return false;
	}

	// Reset the maze for another game;
	private void resetMaze(IRobot robot) {
		for (int i = 1; i < mazeX - 1; i++) {
			for (int j = 1; j < mazeY - 1; j++) {
				robot.getMaze().setCellType(i, j, 1);
			}
		}
	}

	// Manually create all the possible tetriminos
	private void createTetriminoList() {
		int midPointX;

		midPointX = mazeX / 2;
		tetriminoList = new Point[28][4];

		// Line 1
		tetriminoList[0][0] = new Point(midPointX - 2, 1);
		tetriminoList[0][1] = new Point(midPointX - 1, 1);
		tetriminoList[0][2] = new Point(midPointX, 1);
		tetriminoList[0][3] = new Point(midPointX + 1, 1);

		// Line 2
		tetriminoList[1][0] = new Point(midPointX, 1);
		tetriminoList[1][1] = new Point(midPointX, 2);
		tetriminoList[1][2] = new Point(midPointX, 3);
		tetriminoList[1][3] = new Point(midPointX, 4);

		// Line 3
		tetriminoList[2][0] = new Point(midPointX - 2, 1);
		tetriminoList[2][1] = new Point(midPointX - 1, 1);
		tetriminoList[2][2] = new Point(midPointX, 1);
		tetriminoList[2][3] = new Point(midPointX + 1, 1);

		// Line 4
		tetriminoList[3][0] = new Point(midPointX - 1, 1);
		tetriminoList[3][1] = new Point(midPointX - 1, 2);
		tetriminoList[3][2] = new Point(midPointX - 1, 3);
		tetriminoList[3][3] = new Point(midPointX - 1, 4);

		// Reverse L 1
		tetriminoList[4][0] = new Point(midPointX - 1, 1);
		tetriminoList[4][1] = new Point(midPointX - 1, 2);
		tetriminoList[4][2] = new Point(midPointX, 2);
		tetriminoList[4][3] = new Point(midPointX + 1, 2);

		// Reverse L 2
		tetriminoList[5][0] = new Point(midPointX, 1);
		tetriminoList[5][1] = new Point(midPointX + 1, 1);
		tetriminoList[5][2] = new Point(midPointX, 2);
		tetriminoList[5][3] = new Point(midPointX, 3);

		// Reverse L 3
		tetriminoList[6][0] = new Point(midPointX - 1, 1);
		tetriminoList[6][1] = new Point(midPointX, 1);
		tetriminoList[6][2] = new Point(midPointX + 1, 1);
		tetriminoList[6][3] = new Point(midPointX + 1, 2);

		// Reverse L 4
		tetriminoList[7][0] = new Point(midPointX, 1);
		tetriminoList[7][1] = new Point(midPointX, 2);
		tetriminoList[7][2] = new Point(midPointX - 1, 3);
		tetriminoList[7][3] = new Point(midPointX, 3);

		// L 1
		tetriminoList[8][0] = new Point(midPointX + 1, 1);
		tetriminoList[8][1] = new Point(midPointX - 1, 2);
		tetriminoList[8][2] = new Point(midPointX, 2);
		tetriminoList[8][3] = new Point(midPointX + 1, 2);

		// L 2
		tetriminoList[9][0] = new Point(midPointX, 1);
		tetriminoList[9][1] = new Point(midPointX, 2);
		tetriminoList[9][2] = new Point(midPointX, 3);
		tetriminoList[9][3] = new Point(midPointX + 1, 3);

		// L 3
		tetriminoList[10][0] = new Point(midPointX - 1, 1);
		tetriminoList[10][1] = new Point(midPointX, 1);
		tetriminoList[10][2] = new Point(midPointX + 1, 1);
		tetriminoList[10][3] = new Point(midPointX - 1, 2);

		// L 4
		tetriminoList[11][0] = new Point(midPointX - 1, 1);
		tetriminoList[11][1] = new Point(midPointX, 1);
		tetriminoList[11][2] = new Point(midPointX, 2);
		tetriminoList[11][3] = new Point(midPointX, 3);

		// Square 1
		tetriminoList[12][0] = new Point(midPointX - 1, 1);
		tetriminoList[12][1] = new Point(midPointX, 1);
		tetriminoList[12][2] = new Point(midPointX - 1, 2);
		tetriminoList[12][3] = new Point(midPointX, 2);

		// Square 2
		tetriminoList[13][0] = new Point(midPointX - 1, 1);
		tetriminoList[13][1] = new Point(midPointX, 1);
		tetriminoList[13][2] = new Point(midPointX - 1, 2);
		tetriminoList[13][3] = new Point(midPointX, 2);

		// Square 3
		tetriminoList[14][0] = new Point(midPointX - 1, 1);
		tetriminoList[14][1] = new Point(midPointX, 1);
		tetriminoList[14][2] = new Point(midPointX - 1, 2);
		tetriminoList[14][3] = new Point(midPointX, 2);

		// Square 4
		tetriminoList[15][0] = new Point(midPointX - 1, 1);
		tetriminoList[15][1] = new Point(midPointX, 1);
		tetriminoList[15][2] = new Point(midPointX - 1, 2);
		tetriminoList[15][3] = new Point(midPointX, 2);

		// S 1
		tetriminoList[16][0] = new Point(midPointX, 1);
		tetriminoList[16][1] = new Point(midPointX + 1, 1);
		tetriminoList[16][2] = new Point(midPointX - 1, 2);
		tetriminoList[16][3] = new Point(midPointX, 2);

		// S 2
		tetriminoList[17][0] = new Point(midPointX, 1);
		tetriminoList[17][1] = new Point(midPointX, 2);
		tetriminoList[17][2] = new Point(midPointX + 1, 2);
		tetriminoList[17][3] = new Point(midPointX + 1, 3);

		// S 3
		tetriminoList[18][0] = new Point(midPointX, 1);
		tetriminoList[18][1] = new Point(midPointX + 1, 1);
		tetriminoList[18][2] = new Point(midPointX - 1, 2);
		tetriminoList[18][3] = new Point(midPointX, 2);

		// S 4
		tetriminoList[19][0] = new Point(midPointX - 1, 1);
		tetriminoList[19][1] = new Point(midPointX - 1, 2);
		tetriminoList[19][2] = new Point(midPointX, 2);
		tetriminoList[19][3] = new Point(midPointX, 3);

		// K 1
		tetriminoList[20][0] = new Point(midPointX, 1);
		tetriminoList[20][1] = new Point(midPointX - 1, 2);
		tetriminoList[20][2] = new Point(midPointX, 2);
		tetriminoList[20][3] = new Point(midPointX + 1, 2);

		// K 2
		tetriminoList[21][0] = new Point(midPointX, 1);
		tetriminoList[21][1] = new Point(midPointX, 2);
		tetriminoList[21][2] = new Point(midPointX + 1, 2);
		tetriminoList[21][3] = new Point(midPointX, 3);

		// K 3
		tetriminoList[22][0] = new Point(midPointX - 1, 1);
		tetriminoList[22][1] = new Point(midPointX, 1);
		tetriminoList[22][2] = new Point(midPointX + 1, 1);
		tetriminoList[22][3] = new Point(midPointX, 2);

		// K 4
		tetriminoList[23][0] = new Point(midPointX, 1);
		tetriminoList[23][1] = new Point(midPointX - 1, 2);
		tetriminoList[23][2] = new Point(midPointX, 2);
		tetriminoList[23][3] = new Point(midPointX, 3);

		// Z 1
		tetriminoList[24][0] = new Point(midPointX - 1, 1);
		tetriminoList[24][1] = new Point(midPointX, 1);
		tetriminoList[24][2] = new Point(midPointX, 2);
		tetriminoList[24][3] = new Point(midPointX + 1, 2);

		// Z 2
		tetriminoList[25][0] = new Point(midPointX + 1, 1);
		tetriminoList[25][1] = new Point(midPointX, 2);
		tetriminoList[25][2] = new Point(midPointX + 1, 2);
		tetriminoList[25][3] = new Point(midPointX, 3);

		// Z 3
		tetriminoList[26][0] = new Point(midPointX - 1, 1);
		tetriminoList[26][1] = new Point(midPointX, 1);
		tetriminoList[26][2] = new Point(midPointX, 2);
		tetriminoList[26][3] = new Point(midPointX + 1, 2);

		// Z 4
		tetriminoList[27][0] = new Point(midPointX, 1);
		tetriminoList[27][1] = new Point(midPointX - 1, 2);
		tetriminoList[27][2] = new Point(midPointX, 2);
		tetriminoList[27][3] = new Point(midPointX - 1, 3);
	}

	/*
		Main control methods
	*/

	public void reset() {
		pollRun = 0;
		stepCounter = 1;
		// Reset the array
		for (int i = 0; i < mazeGrid.length; i++) {
			for (int j = 0; j < mazeGrid.length; j++) {
				mazeGrid[i][j] = 0;
			}
		}
		// Reset the original starting square and target square (after the games alter them)
		theMaze.setStart(originalStartingSquare.x, originalStartingSquare.y);
		theMaze.setFinish(originalTargetSquare.x, originalTargetSquare.y);
		currentRobotSquare = originalStartingSquare;
		currentTargetSquare = originalTargetSquare;

		// Reset the maze of seen squares used in the Loopy Maze Game Mode
		for (int i = 1; i < mazeX; i++) {
			for (int j = 1; j < mazeY; j++) {
				seenSquare[i][j] = false;
			}
		}
		squaresLeft = passageCounter;

		// If the maze is in Blank Maze Game Mode, clear the maze of the remaining tetrimino squares
		if (robotSearchMode == 5) {
			for (int i = 1; i < mazeX - 1; i++) {
				for (int j = 1; j < mazeY - 1; j++) {
					theMaze.setCellType(i, j, 1);
				}
			}
		}
	}

	public void controlRobot(IRobot robot) {
		int initialHeading = robot.getHeading();
		int robotHeading;

		// Reset the data store on the first move of the first run of a new maze
		if (robot.getRuns() == 0 && pollRun == 0) {
			pollRun = 0;
			mazeGrid = new int[MAX_MAZE_SIZE][MAX_MAZE_SIZE];
			seenSquare = new boolean[MAX_MAZE_SIZE][MAX_MAZE_SIZE];
			junctionStack = new Stack<JunctionRecorder>();
			startingSquare = robot.getLocation();
			robotSearchMode = 0;
			// If the starting square is not a junction, add it to the stack anyway (to ensure that after exploring the maze, the robot gets back to the starting square)
			if (countAdjacentSquares(robot, IRobot.WALL) > 1) {
				mazeGrid[robot.getLocation().x][robot.getLocation().y] = 1;
				junctionStack.push(new JunctionRecorder(robot.getLocation(), IRobot.SOUTH));
			}
			// Game stuff
			originalStartingSquare = robot.getLocation();
			originalTargetSquare = robot.getTargetLocation();
			userInputPanel.setVisible(false);
			theMaze = robot.getMaze();
			mazeIsLoopy = false;
			mazeX = 1;
			mazeY = 1;
			passageCounter = 0;
		}

		// After a few runs, switch to Game mode
		if (robot.getRuns() == 4 && pollRun == 0) {
			for (int i = 1; i <= mazeX; i++) {
				for (int j = 1; j <= mazeY; j++) {
					if (mazeGridFrame[i][j] != -1) {
						passageCounter++;
					}
				}
			}
			// Check if it's a blank maze
			if (passageCounter == mazeX * mazeY) {
				robotSearchMode = 5;
				JOptionPane.showMessageDialog(null, "Let's play Tetris! Use W, A, S, D to move the pieces.");
				robot.getMaze().setStart(0, 0);
				robot.getMaze().setFinish(0, 1);
				robot.setHeading(IRobot.EAST);
				EventBus.broadcast(new Event(107, robot.getMaze()));
				needTetrimino = true;
				mazeX = robot.getMaze().getWidth();
				mazeY = robot.getMaze().getHeight();
				tetrimino = new Point[4];
				for (int i = 0; i <= 3; i++) {
					tetrimino[i] = new Point();
				}
				tetriminoLanded = false;
				createTetriminoList();
			}
			// Check if it's a Prim maze
			else if (mazeIsLoopy == false) {
				robotSearchMode = 3;
				JOptionPane.showMessageDialog(null, "What if the target chased you instead? See how much you can last! Use W, A, S, D to move.");
			}
			// Check if it's a loopy maze
			else {
				robotSearchMode = 4;
				JOptionPane.showMessageDialog(null, "Try to cover all squares before the target catches you! Sort of a Pac-Man! Use W, A, S, D to move.");
				squaresLeft = passageCounter - 1;

			}
			currentTargetSquare = robot.getTargetLocation();
			userInputPanel.setVisible(true);
		}

		if (robot.getRuns() > 4 && pollRun == 0 && robotSearchMode == 5) {
			robot.getMaze().setStart(0, 0);
			robot.getMaze().setFinish(0, 1);
			robot.setHeading(IRobot.EAST);
			EventBus.broadcast(new Event(107, robot.getMaze()));
		}

		pollRun++;

		// Explore Mode
		if (robotSearchMode == 0) {
			// Record surrounding walls in mazeGrid for later use
			for (int heading = IRobot.NORTH; heading <= IRobot.WEST; heading++) {
				if (headingTowardsSquareType(robot, heading, IRobot.WALL) == true) {
					Point testSquare = testCoordinates(robot.getLocation(), heading);
					mazeGrid[testSquare.x][testSquare.y] = -1;
				}
			}
			// If the robot is in a new junction, push it into the junctionStack;
			if (countAdjacentSquares(robot, IRobot.WALL) <= 1 && mazeGrid[robot.getLocation().x][robot.getLocation().y] == 0) {
				mazeGrid[robot.getLocation().x][robot.getLocation().y] = 1;
				junctionStack.push(new JunctionRecorder(robot.getLocation(), initialHeading));
			}
			robotHeading = exploreControl(robot);

			if (testCoordinates(robot.getLocation(), robotHeading).x > mazeX) {
				mazeX = testCoordinates(robot.getLocation(), robotHeading).x;
			}

			if (testCoordinates(robot.getLocation(), robotHeading).y > mazeY) {
				mazeY = testCoordinates(robot.getLocation(), robotHeading).y;
			}
		}
		// Backtrack Mode
		else if (robotSearchMode == 1) {
			robotHeading = backtrackControl(robot);
		}
		// Shortest Path Mode
		else if (robotSearchMode == 2) {
			robotHeading = shortestPath[stepCounter++];
		}
		// Prim Maze Game Mode
		else if (robotSearchMode == 3) {
			robotHeading = primMazeGame(robot, 107);
		}
		// Loopy Maze Game Mode
		else if (robotSearchMode == 4) {
			robotHeading = loopyMazeGame(robot);
			// "Abduct" the robot if it's next to the target
			for (int heading = IRobot.NORTH; heading <= IRobot.WEST; heading++) {
				if (testCoordinates(currentRobotSquare, heading).equals(currentTargetSquare) == true) {
					robotHeading = heading;
					EventBus.broadcast(new Event(107, robot.getMaze()));
					break;
				}
			}
		}
		// Blank Maze Game Mode
		else {
			robotHeading = IRobot.EAST;
			if (gameOver(robot) == false || tetriminoLanded(robot) == false) {
				blankMazeGame(robot);
			}
			else {
				JOptionPane.showMessageDialog(null, "Game over. Start again?");
				resetMaze(robot);
				EventBus.broadcast(new Event(107, robot.getMaze()));
			}
		}
		// If the robot is still exploring the maze and is facing the target, turn back
		if (robotSearchMode < 2 && testCoordinates(robot.getLocation(), robotHeading).equals(robot.getTargetLocation()) == true) {
			robotHeading = reverseHeading(initialHeading);
		}
		robot.setHeading(robotHeading);
	}
}

// Class for game controller
class InputPanel extends JFrame implements KeyListener {
	private int heading = IRobot.NORTH;
	private char tetrisInput = 'S';
	private JTextPane inputTextPane;

	public InputPanel() {
		super("Input Panel");
		// Create a text pane that looks like a standard console
		setBounds(0, 0, 250, 150);
		setDefaultCloseOperation(EXIT_ON_CLOSE);
		inputTextPane = new JTextPane();
		inputTextPane.setBackground(Color.BLACK);
		inputTextPane.setForeground(Color.LIGHT_GRAY);
		inputTextPane.setFont(new Font("courier", Font.BOLD, 12));
		inputTextPane.addKeyListener(this);
		getContentPane().add(inputTextPane);
    }

	public void keyPressed(KeyEvent arg0)  {
		tetrisInput = arg0.getKeyChar();
	}

	// Leave keyReleased blank, since it is not needed (must be implemeneted, though)
	public void keyReleased(KeyEvent arg0)  {}

	// Get the user's input and transform it into a heading (if input is valid)
	public void keyTyped(KeyEvent arg0) {
		char userInput = arg0.getKeyChar();

		if (userInput == 'w' || userInput == 'W') {
			heading = IRobot.NORTH;
		}
		else if (userInput == 'a' || userInput == 'A') {
			heading = IRobot.WEST;
		}
		else if (userInput == 's' || userInput == 'S') {
			heading = IRobot.SOUTH;
		}
		else if (userInput == 'd' || userInput == 'D') {
			heading = IRobot.EAST;
		}
    }

    public int getInputHeading() {
    	return heading;
    }

    public char getTetrisInput() {
    	char tetrisInputCopy = tetrisInput;
    	// Switch it to a non-WASD character so the tetrimino doesn't keep moving in a direction until it gets a new input
    	tetrisInput = 'X';
    	return tetrisInputCopy;
    }
}
