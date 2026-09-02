/**
 * Deterministic star field for the optical viewport background.
 * Fixed coordinate list — NO Math.random (matches the no-fake-state rule and
 * keeps every render/screenshot identical).
 */
const STARS: Array<[number, number, number]> = [
  [4, 9, 0.5], [11, 71, 0.6], [17, 33, 1], [23, 88, 0.5], [29, 12, 0.8],
  [31, 54, 0.6], [37, 77, 1], [41, 22, 0.5], [43, 63, 0.7], [47, 91, 0.6],
  [52, 8, 0.9], [56, 44, 0.5], [58, 82, 0.6], [61, 27, 1.1], [64, 68, 0.6],
  [67, 15, 0.5], [71, 55, 0.8], [73, 93, 0.6], [77, 37, 0.6], [79, 74, 1],
  [82, 19, 0.5], [85, 61, 0.7], [88, 45, 0.6], [91, 84, 0.9], [93, 29, 0.5],
  [96, 66, 0.6], [8, 48, 0.7], [14, 96, 0.5], [19, 58, 0.6], [26, 41, 0.8],
  [34, 6, 0.5], [39, 39, 0.6], [45, 79, 0.7], [50, 25, 0.5], [54, 97, 0.6],
  [63, 51, 0.9], [69, 84, 0.5], [75, 4, 0.6], [86, 8, 0.7], [98, 15, 0.5],
  [2, 27, 0.6], [12, 18, 0.9], [22, 66, 0.5], [33, 30, 0.6], [48, 55, 0.7],
  [59, 12, 0.5], [66, 40, 0.6], [80, 52, 0.8], [90, 71, 0.5], [95, 46, 0.6],
];

export function Starfield({ tint = "#0a0f0e" }: { tint?: string }) {
  return (
    <div className="absolute inset-0" aria-hidden>
      <div
        className="absolute inset-0"
        style={{
          background: `radial-gradient(ellipse at 50% 45%, ${tint} 0%, #060807 78%)`,
        }}
      />
      <svg className="absolute inset-0 h-full w-full" viewBox="0 0 100 100" preserveAspectRatio="none">
        {STARS.map(([x, y, r], i) => (
          <circle
            key={i}
            cx={x}
            cy={y}
            r={r * 0.12}
            fill="#dee4e2"
            fillOpacity={0.15 + (r % 1) * 0.5}
          />
        ))}
      </svg>
    </div>
  );
}
