package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;
import java.util.List;

import com.google.gson.Gson;

import org.firstinspires.ftc.teamcode.models.Coordinate;
import org.firstinspires.ftc.teamcode.models.Field;

public class GridBuilder {
    // In inches, uh i kinda calculated this by taking the length of the field in feet
    // divide that by the length of the ball or soemthing and thats what size i got
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

    /*
    public Field createEmptyField() throws Exception {
        for (int i = 1; i <= GRID_LENGTH; i++) {
            ArrayList<Integer> arrList = new ArrayList<>();

            for (int l = 1; l <= GRID_WIDTH; l++) {
                arrList.add(0);
            }

            matrix.add(new ArrayList<>());
        }

        return new Field();
    }
    */

    public Field build() {
        List<Object[]> list = new ArrayList<>();
        
        for (ArrayList<Integer> integers : matrix) {
            Object[] toArray = integers.toArray();
            list.add(toArray);
        }
        int[][] matrix_ = (int[][]) list.toArray();

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
