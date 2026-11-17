/**
 * Purpose: the three part materials (steel, aluminium, carbon fibre) and what a member of one of
 * them is for the core: stiffness `EA`, yield and break strain, mass per metre, and the damping
 * that gives a chosen damping ratio.
 * Value: every generator takes its numbers from here, so "members yield and break at real forces"
 * is stated once (ER 004, numerics) and tested once (test/parts.test.ts), and a change to the
 * stiffness scale is one line.
 *
 * Real densities and real yield and break stresses, Young's modulus multiplied by one common scale
 * `STIFFNESS_SCALE` so the stiffest members fit the substep budget. Strains come from the real
 * stresses and the scaled modulus (`yield strain = σ_y / (s·E)`), so a member's yield force is
 * `s·E·A · σ_y / (s·E) = σ_y · A`, whatever the scale: only the stretch on the way there is `1/s`
 * times longer. Only `+ − × ÷` and `sqrt` appear here (generator rule: every engine gives the same
 * bits).
 */

/** The common factor on every Young's modulus **[assumption]** (ER 004, numerics). */
export const STIFFNESS_SCALE = 0.1;

export type MaterialId = 'steel' | 'aluminium' | 'carbon';

export interface PartMaterial {
  id: MaterialId;
  /** Real Young's modulus in pascals (the scale is applied in [`member`]). */
  youngPa: number;
  /** Density in kilograms per cubic metre. */
  density: number;
  /** Real yield stress in pascals, or `null` for a brittle material that breaks without yielding. */
  yieldPa: number | null;
  /** Total strain at which a member breaks. Game values for steel and aluminium (ER 004). */
  breakStrain: number;
}

/**
 * ER 004's table. Break strains of steel (0.10, ductile, about 6 × yield) and aluminium (0.08,
 * about 3 × yield) are game values, not real elongation at fracture. Carbon fibre's 0.05 is its
 * 600 MPa strength over the scaled modulus (`600e6 / 12e9`), so it breaks at a real force.
 */
export const MATERIALS: Readonly<Record<MaterialId, PartMaterial>> = {
  steel: { id: 'steel', youngPa: 200e9, density: 7850, yieldPa: 350e6, breakStrain: 0.1 },
  aluminium: { id: 'aluminium', youngPa: 70e9, density: 2700, yieldPa: 200e6, breakStrain: 0.08 },
  carbon: { id: 'carbon', youngPa: 120e9, density: 1600, yieldPa: null, breakStrain: 0.05 },
};

/** A brittle material's yield strain, as a multiple of its break strain: set past it, never reached. */
export const BRITTLE_YIELD_FACTOR = 1.2;

/** What the core needs to know about one member (a beam of a given material and cross-section area). */
export interface Member {
  material: MaterialId;
  /** Cross-section area in square metres. */
  area: number;
  /** Axial stiffness `s · E · A` in newtons; a beam's compliance is `rest / ea`. */
  ea: number;
  yieldStrain: number;
  breakStrain: number;
  /** Axial force at which the member yields, `σ_y · A`; `null` for a brittle material. */
  yieldForce: number | null;
  /** Mass per metre of length, `density · A`, in kilograms. */
  massPerLength: number;
}

/** A member's own numbers, from its material and cross-section area. */
export function member(material: MaterialId, area: number): Member {
  const m = MATERIALS[material];
  if (m === undefined) throw new Error(`unknown material "${String(material)}"`);
  if (!(area > 0) || !Number.isFinite(area)) throw new Error(`a member's area must be a positive number, not ${area}`);
  const ea = STIFFNESS_SCALE * m.youngPa * area;
  const yieldStrain = m.yieldPa === null ? m.breakStrain * BRITTLE_YIELD_FACTOR : m.yieldPa / (STIFFNESS_SCALE * m.youngPa);
  return {
    material,
    area,
    ea,
    yieldStrain,
    breakStrain: m.breakStrain,
    yieldForce: m.yieldPa === null ? null : m.yieldPa * area,
    massPerLength: m.density * area,
  };
}

/**
 * Mass two ends act as when they move against each other: `m_a · m_b / (m_a + m_b)`. A pinned
 * end is infinitely heavy (`Infinity`), so the other end's mass is the answer; two pinned ends
 * give zero.
 */
export function reducedMass(massA: number, massB: number): number {
  if (massA === Infinity) return massB === Infinity ? 0 : massB;
  if (massB === Infinity) return massA;
  return (massA * massB) / (massA + massB);
}

/**
 * Damping in newton-seconds per metre that gives a beam the damping ratio `zeta` against its two
 * end masses: `c = ζ · 2 · sqrt(k · m_a·m_b / (m_a + m_b))` with `k = EA / L`. At `ζ = 1` the two
 * ends return to rest without overshoot; the spec's minimum for structure is 0.05.
 */
export function dampingFor(ea: number, length: number, massA: number, massB: number, zeta: number): number {
  const k = ea / length;
  return zeta * 2 * Math.sqrt(k * reducedMass(massA, massB));
}
