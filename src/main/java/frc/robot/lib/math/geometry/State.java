package frc.robot.lib.math.geometry;

import frc.robot.lib.util.Interpolable;
import frc.robot.lib.util.CSVWritable;

public interface State<S> extends Interpolable<S>, CSVWritable {
    double distance(final S other);

    boolean equals(final Object other);

    String toString();

    String toCSV();
}