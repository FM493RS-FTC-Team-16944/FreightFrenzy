package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;


public class twoDGridPtf {
/*
    public static int[][] matArray = {{1, 1, 1, 1}, {0, 0, 0, 0}, {1, 0, 1, 0}} //just an example of what the array would look like, not actual
    private HashMap<Integer, ArrayList<String>> possPaths = new HashMap<Integer, ArrayList<String>>();
    private final int count = 0;

    /**public ArrayList<String> returnPath(int x, int y) {
        possPaths = new HashMap<Integer, ArrayList<String>>();
        int currentPosition = (0) ( int)//call localization
        int currentX;
        int currentY;
        for (int i = 0; i < matArray.length; i++) {
            for (int j = 0; j < matArray[i].length) {
                int distanceSqr = (Math.sqrt(i * i + j * j)) ( int)
                if (currentPosition == distanceSqr) {
                    currentX = i;
                    currentY = j;
                    break;
                }
            }
        }
        ArrayList<String> path = new ArrayList<String>();
        findSolutions(currentX, currentY, x, y, path, 0);
        //loop through solutions and pass the shortest one
        double shortDistance = Double.POSITIVE_INFINITY;
        ArrayList<String> shortestPath;
        for (Map.Entry<String, String> entry : possPaths.entrySet()) {
            if (entry.key < shortDistance) {
                shortDistance = entry.key;
                shortestPath = entry.value;
            }
        }
    }

    public void findSolutions(int onX, int onY, int targetX, int targetY, ArrayList<String> path, double distanceTraveled) {
        ArrayList<String> newPath = new ArrayList<String>(path);
        newPath.add(onX + "," + onY);
        if (targetX == onX && targetY == onY) {
            possPaths.put(distanceTraveled, newPath);
        } else if (matArray[onX][onY] == 1) {
            return;
        } else {
            for (int i = -1; i <= 1; i++) {
                for (int j = -1; j <= 1) {
                    double newTraveled = distanceTraveled + newDistance; //need to figure out how to track distance traveled
                    findSolutions(onX + i, onY + j, targetX, targetY, newPath, distanceTraveled);
                }
            }
        }
    }
    */
}
