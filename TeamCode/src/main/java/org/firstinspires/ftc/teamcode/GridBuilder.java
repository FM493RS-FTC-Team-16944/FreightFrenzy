package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;
import com.google.gson.Gson;

import org.firstinspires.ftc.teamcode.models.Coordinate;
import org.firstinspires.ftc.teamcode.models.Field;

public class GridBuilder {
    final int GRID_LENGTH = 48;
    final int GRID_WIDTH = 48;

    public static final Gson gson = new Gson();

    private final ArrayList<ArrayList<Integer>> matrix = new ArrayList<>();

    public static Field deserializeField(String json) {
        return gson.fromJson(json, Field.class);
    }

    public String serializeField() {
        return gson.toJson(this.matrix);
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
