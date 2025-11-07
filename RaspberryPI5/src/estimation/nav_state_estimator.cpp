/*Fuses heading (IMU if present, else COG fallback), SOG, position.
Computes True Wind Direction (TWD) and True Wind Speed (TWS) from AWA + boat velocity.
Publishes a single NavState message: {lat, lon, heading, sog, cog, awa, twd, tws}.
*/