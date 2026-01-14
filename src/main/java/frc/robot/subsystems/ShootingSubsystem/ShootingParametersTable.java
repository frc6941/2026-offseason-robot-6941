package frc.robot.subsystems.ShootingSubsystem;

import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.Map;
import java.util.Map.Entry;
import java.util.NavigableMap;
import java.util.Set;
import java.util.TreeMap;
import java.util.TreeSet;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import frc.robot.subsystems.shooter.ShootingPresetParamsNT;
import lib.ntext.NTParameterWrapper;

/**
 * Distance-indexed lookup table of shooter parameters, sourced from {@code ShooterTableParamsNT}.
 *
 * <p>Call {@link #updateFromNT()} after {@code NTParameterRegistry.refresh()} to pull latest tuning
 * values into the interpolating map.
 */
public class ShootingParametersTable {
    private final NavigableMap<Double, ShootingParameters> interpolatingTable = new TreeMap<>();
    private static final Pattern kPointFieldPattern = Pattern.compile("^P(\\d+)_(DIS|FWV|BBA)$");

    public ShootingParametersTable() {
        updateFromNT();
    }

    /** Rebuilds the interpolating table from the latest NTParameter wrapper values. */
    public synchronized void updateFromNT() {
        interpolatingTable.clear();

        // Avoid having to duplicate the point list here: discover all P#_(DIS|FWV|BBA) fields on
        // the
        // generated ShooterTableParamsNT class.
        Map<Integer, NTParameterWrapper<Double>> dis = new HashMap<>();
        Map<Integer, NTParameterWrapper<Double>> fwv = new HashMap<>();
        Map<Integer, NTParameterWrapper<Double>> bba = new HashMap<>();
        Set<Integer> indices = new TreeSet<>();

        // IMPORTANT: reflect over the generated NT wrapper class (public static fields).
        for (Field field : ShootingPresetParamsNT.class.getFields()) {
            Matcher m = kPointFieldPattern.matcher(field.getName());
            if (!m.matches()) continue;

            int i = Integer.parseInt(m.group(1));
            String kind = m.group(2);
            NTParameterWrapper<Double> wrapper = getDoubleWrapper(field);
            if (wrapper == null) continue;

            indices.add(i);
            switch (kind) {
                case "DIS" -> dis.put(i, wrapper);
                case "FWV" -> fwv.put(i, wrapper);
                case "BBA" -> bba.put(i, wrapper);
                default -> {
                    // Unreachable: regex only allows DIS/FWV/BBA
                }
            }
        }

        for (int i : indices) {
            NTParameterWrapper<Double> disW = dis.get(i);
            NTParameterWrapper<Double> fwvW = fwv.get(i);
            NTParameterWrapper<Double> bbaW = bba.get(i);
            if (disW == null || fwvW == null || bbaW == null) continue;

            putPoint(disW.getValue(), fwvW.getValue(), bbaW.getValue());
        }
    }

    private void putPoint(double distance, double velocityRpm, double backboardAngleDegrees) {
        interpolatingTable.put(
                distance, new ShootingParameters(velocityRpm, backboardAngleDegrees));
    }

    @SuppressWarnings("unchecked")
    private static NTParameterWrapper<Double> getDoubleWrapper(Field field) {
        try {
            Object v = field.get(null);
            if (v instanceof NTParameterWrapper<?> w) {
                return (NTParameterWrapper<Double>) w;
            }
        } catch (IllegalAccessException ignored) {
            // Public fields should be accessible; ignore if not.
        }
        return null;
    }

    /** Returns interpolated parameters at {@code distance}, clamped to the table endpoints. */
    public synchronized ShootingParameters getParameters(double distance) {
        if (interpolatingTable.isEmpty()) {
            return new ShootingParameters(0.0, 0.0);
        }

        if (distance <= interpolatingTable.firstKey()) {
            return interpolatingTable.firstEntry().getValue();
        }

        if (distance >= interpolatingTable.lastKey()) {
            return interpolatingTable.lastEntry().getValue();
        }

        Entry<Double, ShootingParameters> floor = interpolatingTable.floorEntry(distance);
        Entry<Double, ShootingParameters> ceiling = interpolatingTable.ceilingEntry(distance);

        if (floor == null) {
            return interpolatingTable.firstEntry().getValue();
        }
        if (ceiling == null) {
            return interpolatingTable.lastEntry().getValue();
        }
        if (floor.getKey().equals(ceiling.getKey())) {
            return floor.getValue();
        }

        double t = (distance - floor.getKey()) / (ceiling.getKey() - floor.getKey());
        return floor.getValue().interpolate(ceiling.getValue(), t);
    }

    /** Custom preset (standalone, not part of interpolation table). */
    public synchronized ShootingParameters getCustomShotParameters() {
        return new ShootingParameters(
                ShootingPresetParamsNT.customFWV.getValue(),
                ShootingPresetParamsNT.customBBA.getValue());
    }
}
