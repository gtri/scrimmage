/**
 * Reverse-geocode a (lat, lon) to a human-friendly location name using OpenStreetMap
 * Nominatim. Free, no API key. Their usage policy rate-limits to ~1 req/sec — fine
 * for our use (we only call once per mission start).
 *
 * Returns null on any failure (network, parse, no result) so callers can fall back
 * to showing just the coordinates.
 */
export async function reverseGeocode(lat: number, lon: number): Promise<string | null> {
  try {
    // zoom=16 prioritizes specific named places (military bases, parks, neighborhoods)
    // over broader admin regions. Lower zoom = broader region, higher = more specific.
    const url = `https://nominatim.openstreetmap.org/reverse?lat=${lat}&lon=${lon}&format=json&zoom=16`;
    const r = await fetch(url, { headers: { Accept: 'application/json' } });
    if (!r.ok) return null;
    const data = await r.json();
    return pickShortName(data.address || {});
  } catch {
    return null;
  }
}

/**
 * Pick a sensible short label from a Nominatim address response. Prefers specific
 * named places (military bases, towns, attractions) over administrative regions,
 * then pairs with state/country for context.
 */
function pickShortName(address: Record<string, string>): string | null {
  const primary =
    address.military ||
    address.tourism ||
    address.attraction ||
    address.city ||
    address.town ||
    address.village ||
    address.hamlet ||
    address.suburb ||
    address.neighbourhood ||
    address.county;
  const region = address.state || address.country;
  if (primary && region && primary !== region) return `${primary}, ${region}`;
  return primary || region || null;
}
