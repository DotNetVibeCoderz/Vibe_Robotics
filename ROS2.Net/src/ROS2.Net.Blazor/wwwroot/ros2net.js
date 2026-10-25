// Tiny helpers for ROS2.Net.Blazor components (element geometry and pointer capture).
export function rect(el) {
  if (!el) return { width: 0, height: 0 };
  const r = el.getBoundingClientRect();
  return { width: r.width, height: r.height };
}

export function capture(el, pointerId) {
  try { el?.setPointerCapture(pointerId); } catch { /* element gone */ }
}

export function release(el, pointerId) {
  try { el?.releasePointerCapture(pointerId); } catch { /* not captured */ }
}

// Prevent page scroll while zooming a map with the wheel.
export function trapWheel(el) {
  el?.addEventListener('wheel', e => e.preventDefault(), { passive: false });
}
