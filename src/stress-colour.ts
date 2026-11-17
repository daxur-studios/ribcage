/**
 * Purpose: the colour a beam is drawn in for its stress, one scale for the 2D editor and the 3D page.
 * Value: a person who learns to read the colours in one view reads them the same in the other, and
 * the scale is plain arithmetic a test can pin.
 *
 * `ratio` is strain over yield strain: positive is tension (towards red), negative is compression
 * (towards blue), and both fade to grey at zero. A beam bent past yield and a broken beam have
 * their own colours.
 */

export type Rgb = readonly [number, number, number];

export const STRESS_GREY: Rgb = [138, 147, 163];
export const STRESS_RED: Rgb = [255, 93, 93];
export const STRESS_BLUE: Rgb = [74, 163, 255];
/** A beam that has bent past yield (permanent set). */
export const YIELDED_RGB: Rgb = [217, 107, 255];
/** A broken beam (the 2D editor draws it dashed; the 3D page hides it). */
export const BROKEN_RGB: Rgb = [85, 92, 105];

/** The colour for a stress ratio, as integers 0 to 255. Grey at 0, full red at +1 and beyond, full blue at -1 and beyond. */
export function stressRgb(ratio: number): [number, number, number] {
  const t = Math.min(1, Math.abs(ratio));
  const to = ratio >= 0 ? STRESS_RED : STRESS_BLUE;
  return [
    Math.round(STRESS_GREY[0] + (to[0] - STRESS_GREY[0]) * t),
    Math.round(STRESS_GREY[1] + (to[1] - STRESS_GREY[1]) * t),
    Math.round(STRESS_GREY[2] + (to[2] - STRESS_GREY[2]) * t),
  ];
}

/** `stressRgb` as a CSS colour. */
export function stressCss(ratio: number): string {
  const [r, g, b] = stressRgb(ratio);
  return `rgb(${r},${g},${b})`;
}
