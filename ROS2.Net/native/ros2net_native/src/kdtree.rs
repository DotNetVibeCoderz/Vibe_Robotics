//! Exact k-d tree over f32 xyz points. Results are ordered by (squared distance, index), which makes
//! them identical to the managed implementation regardless of tree shape.

pub struct KdTree {
    pts: Vec<[f32; 3]>,
    // Implicit balanced tree: `idx` is permuted so that the median of every range sits at its middle.
    idx: Vec<u32>,
    axis: Vec<u8>,
}

#[inline]
pub fn dist2(a: &[f32; 3], b: &[f32; 3]) -> f32 {
    let dx = a[0] - b[0];
    let dy = a[1] - b[1];
    let dz = a[2] - b[2];
    dx * dx + dy * dy + dz * dz
}

#[inline]
fn better(d: f32, i: u32, od: f32, oi: u32) -> bool {
    d < od || (d == od && i < oi)
}

impl KdTree {
    pub fn new(xyz: &[f32]) -> KdTree {
        let n = xyz.len() / 3;
        let mut pts = Vec::with_capacity(n);
        let mut idx = Vec::with_capacity(n);
        for i in 0..n {
            let p = [xyz[3 * i], xyz[3 * i + 1], xyz[3 * i + 2]];
            pts.push(p);
            if p[0].is_finite() && p[1].is_finite() && p[2].is_finite() {
                idx.push(i as u32);
            }
        }
        let m = idx.len();
        let mut tree = KdTree { pts, idx, axis: vec![0; m] };
        tree.build(0, m);
        tree
    }

    pub fn len(&self) -> usize {
        self.pts.len()
    }

    pub fn point(&self, i: usize) -> &[f32; 3] {
        &self.pts[i]
    }

    fn build(&mut self, lo: usize, hi: usize) {
        if hi <= lo + 1 {
            return;
        }
        // Split on the axis of widest spread.
        let mut mn = [f32::INFINITY; 3];
        let mut mx = [f32::NEG_INFINITY; 3];
        for &i in &self.idx[lo..hi] {
            let p = self.pts[i as usize];
            for a in 0..3 {
                mn[a] = mn[a].min(p[a]);
                mx[a] = mx[a].max(p[a]);
            }
        }
        let mut ax = 0;
        for a in 1..3 {
            if mx[a] - mn[a] > mx[ax] - mn[ax] {
                ax = a;
            }
        }
        let mid = (lo + hi) / 2;
        let pts = &self.pts;
        self.idx[lo..hi].select_nth_unstable_by(mid - lo, |&a, &b| {
            pts[a as usize][ax]
                .partial_cmp(&pts[b as usize][ax])
                .unwrap_or(std::cmp::Ordering::Equal)
                .then(a.cmp(&b))
        });
        self.axis[mid] = ax as u8;
        self.build(lo, mid);
        self.build(mid + 1, hi);
    }

    /// k nearest neighbours of `q`, sorted by (distance², index).
    pub fn knn(&self, q: &[f32; 3], k: usize, out: &mut Vec<(f32, u32)>) {
        out.clear();
        if k == 0 {
            return;
        }
        self.knn_rec(0, self.idx.len(), q, k, out);
        out.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap().then(a.1.cmp(&b.1)));
    }

    fn worst(out: &[(f32, u32)]) -> usize {
        let mut w = 0;
        for j in 1..out.len() {
            if better(out[w].0, out[w].1, out[j].0, out[j].1) {
                w = j;
            }
        }
        w
    }

    fn knn_rec(&self, lo: usize, hi: usize, q: &[f32; 3], k: usize, out: &mut Vec<(f32, u32)>) {
        if hi <= lo {
            return;
        }
        let mid = (lo + hi) / 2;
        let i = self.idx[mid];
        let p = &self.pts[i as usize];
        let d = dist2(p, q);
        if out.len() < k {
            out.push((d, i));
        } else {
            let w = Self::worst(out);
            if better(d, i, out[w].0, out[w].1) {
                out[w] = (d, i);
            }
        }
        if hi == lo + 1 {
            return;
        }
        let ax = self.axis[mid] as usize;
        let diff = q[ax] - p[ax];
        let (first, second) = if diff <= 0.0 { ((lo, mid), (mid + 1, hi)) } else { ((mid + 1, hi), (lo, mid)) };
        self.knn_rec(first.0, first.1, q, k, out);
        let plane = diff * diff;
        let need = out.len() < k || plane <= out[Self::worst(out)].0;
        if need {
            self.knn_rec(second.0, second.1, q, k, out);
        }
    }

    /// All points within `radius` of `q` (distance² ≤ radius²), sorted by (distance², index).
    pub fn radius(&self, q: &[f32; 3], radius: f32, out: &mut Vec<(f32, u32)>) {
        out.clear();
        let r2 = radius * radius;
        self.radius_rec(0, self.idx.len(), q, r2, out);
        out.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap().then(a.1.cmp(&b.1)));
    }

    fn radius_rec(&self, lo: usize, hi: usize, q: &[f32; 3], r2: f32, out: &mut Vec<(f32, u32)>) {
        if hi <= lo {
            return;
        }
        let mid = (lo + hi) / 2;
        let i = self.idx[mid];
        let p = &self.pts[i as usize];
        let d = dist2(p, q);
        if d <= r2 {
            out.push((d, i));
        }
        if hi == lo + 1 {
            return;
        }
        let ax = self.axis[mid] as usize;
        let diff = q[ax] - p[ax];
        let (first, second) = if diff <= 0.0 { ((lo, mid), (mid + 1, hi)) } else { ((mid + 1, hi), (lo, mid)) };
        self.radius_rec(first.0, first.1, q, r2, out);
        if diff * diff <= r2 {
            self.radius_rec(second.0, second.1, q, r2, out);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn brute(xyz: &[f32], q: &[f32; 3], k: usize) -> Vec<(f32, u32)> {
        let mut v: Vec<(f32, u32)> = (0..xyz.len() / 3)
            .map(|i| (dist2(&[xyz[3 * i], xyz[3 * i + 1], xyz[3 * i + 2]], q), i as u32))
            .collect();
        v.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap().then(a.1.cmp(&b.1)));
        v.truncate(k);
        v
    }

    #[test]
    fn knn_matches_brute_force() {
        let mut s = 12345u64;
        let mut xyz = Vec::new();
        for _ in 0..3000 {
            s = s.wrapping_mul(6364136223846793005).wrapping_add(1);
            xyz.push(((s >> 40) % 1000) as f32 / 100.0);
        }
        let t = KdTree::new(&xyz);
        let mut out = Vec::new();
        for qi in 0..50 {
            let q = [xyz[3 * qi] + 0.01, xyz[3 * qi + 1], xyz[3 * qi + 2] - 0.02];
            t.knn(&q, 7, &mut out);
            assert_eq!(out, brute(&xyz, &q, 7));
        }
    }

    #[test]
    fn radius_finds_everything_inside() {
        let xyz = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.5, 0.5, 0.0];
        let t = KdTree::new(&xyz);
        let mut out = Vec::new();
        t.radius(&[0.0, 0.0, 0.0], 1.0, &mut out);
        let ids: Vec<u32> = out.iter().map(|x| x.1).collect();
        assert_eq!(ids, vec![0, 3, 1]);
    }
}
