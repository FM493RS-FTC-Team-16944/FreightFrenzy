package org.firstinspires.ftc.teamcode.models;

public class Field {
    final int[][] matrix;
    final int columns;
    final int rows;

    public Field(int[][] matrix) throws Exception {
        this.matrix = matrix;

        for (int i = 0; i < matrix.length - 1; i++) {
            if(matrix[i].length != matrix[i + 1].length)
                throw new Exception();
        }

        this.columns = this.matrix[0].length;
        this.rows = this.matrix.length;
    }

    private boolean isCoordinateMatrix(Coordinate coordinate) {
        return coordinate.x >= 0 && coordinate.y >= 0
                && coordinate.x < this.columns && coordinate.y < this.rows;
    }

    public boolean checkObject(Coordinate coordinate) {
        if(isCoordinateMatrix(coordinate)) {
            return this.matrix[coordinate.y][coordinate.x] != 1;
        }

        return false;
    }
}
