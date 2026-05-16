package org.firstinspires.ftc.teamcode;

public class SwerveKinematics {
    private final int numModules;
    private final double[] moduleHeadings;
    private final Matrix inverseKinematics;

    public SwerveKinematics(Vector... moduleTranslations) {
        numModules = moduleTranslations.length;
        moduleHeadings = new double[numModules];

        inverseKinematics = new Matrix(numModules * 2, 3);
        for (int i = 0; i < numModules; i++) {
            inverseKinematics.setRow(i * 2, 1, 0, -moduleTranslations[i].y);
            inverseKinematics.setRow(i * 2 + 1, 0, 1, moduleTranslations[i].x);
        }
    }

    public SwerveModuleState[] toModuleStates(Vector where, double angleRadians) {
        SwerveModuleState[] resultStates = new SwerveModuleState[numModules];
        if (where.magnitude() < 0.01 && angleRadians == 0) {
            for (int pop = 0; pop < numModules; pop++) {
                resultStates[pop] = new SwerveModuleState(0, moduleHeadings[pop]);
            }
            return resultStates;
        }

        Matrix chassisSpeedsVec = new Matrix(3, 1);
        chassisSpeedsVec.setColumn(0, where.x, where.y, angleRadians);
        Matrix newModuleStatesMatrix = inverseKinematics.multiply(chassisSpeedsVec);

        for (int moduleIndex = 0; moduleIndex < numModules; moduleIndex++) {
            double x = newModuleStatesMatrix.get(moduleIndex * 2, 0);
            double y = newModuleStatesMatrix.get(moduleIndex * 2 + 1, 0);
            Vector vec = new Vector(x, y);

            double speed = vec.magnitude();
            double angle = speed > 0.01 ? vec.angle() : moduleHeadings[moduleIndex];

            resultStates[moduleIndex] = new SwerveModuleState(speed, angle);
            moduleHeadings[moduleIndex] = angle;
        }

        return resultStates;
    }

    private static class Matrix {
        private final double[][] vals;
        private final int rows;
        private final int columns;

        public Matrix(int rows, int columns) {
            vals = new double[rows][columns];
            this.rows = rows;
            this.columns = columns;
        }

        public void setRow(int row, double... newVals) {
            vals[row] = newVals;
        }

        public void setColumn(int col, double... newVals) {
            for (int row = 0; row < rows; row++) {
                vals[row][col] = newVals[row];
            }
        }

        public double get(int row, int col) {
            return vals[row][col];
        }

        public void inc(int row, int col, double val) {
            vals[row][col] += val;
        }

        public Matrix multiply(Matrix other) {
            int resultCols = other.columns;
            Matrix result = new Matrix(rows, resultCols);

            for (int row = 0; row < rows; row++) {
                for (int resultCol = 0; resultCol < resultCols; resultCol++) {
                    for (int col = 0; col < columns; col++) {
                        result.inc(row, resultCol, this.get(row, col) * other.get(col, resultCol));
                    }
                }
            }

            return result;
        }
    }

    public static class SwerveModuleState {
        public double speed;
        public double angle;

        public SwerveModuleState(double speed, double angle) {
            this.speed = speed;
            this.angle = angle;
        }

        public static void desaturate(SwerveModuleState[] states, double limit) {
            double max = 0;
            for (SwerveModuleState state : states) {
                max = Math.max(max, Math.abs(state.speed));
            }
            if (max > limit) {
                for (SwerveModuleState state : states) {
                    state.speed = state.speed / max * limit;
                }
            }
        }
    }
}
