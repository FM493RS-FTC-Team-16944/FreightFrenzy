package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;
import com.google.gson.Gson;

public class GridBuilder {
    final int GRID_LENGTH = 24;
    final int GRID_WIDTH = 24;

    private final ArrayList<ArrayList<Integer>> matrix = new ArrayList<>();

    public static Field deserializeField(String json) {
        return Gson.fromJson(json, Field.class);
    }

    public Gson serializeField() {
        return Gson.toJson(this.matrix);
    }

    public Field build() {
        int[][] matrix_ = (int[][]) matrix.stream().map(ArrayList::toArray).toArray();

        try {
            return new Field(matrix_);
        } catch (Exception e) {
            System.out.println("uneven field");
            e.printStackTrace();

            return null;
        }
    }

    public void listener(Coordinate coordinate, boolean isObject) {
        int object = isObject ? 1 : 0;

        ArrayList<Integer> arr;

        if(matrix.get(coordinate.y) == null) {
            arr = new ArrayList<>();

            matrix.add(coordinate.y, arr);
        } else {
            arr = matrix.get(coordinate.y);
        }

        arr.add(coordinate.x, object);
    }
}
