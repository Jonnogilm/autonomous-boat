/*IDLE → ARM → SAIL → (TACK/JIBE/LOITER/HEAVE_TO) → RETURN_TO_HOME → FAULT
Entry/exit guards, timeouts, and clear transitions.
Safety hooks:
Geofence (lat/lon polygon or radius) → RTB
Loss of GNSS/heading/wind → HEAVE_TO (fail-safe)
Low battery / overcurrent → LOITER / RTB
Manual override channel → IDLE
*/