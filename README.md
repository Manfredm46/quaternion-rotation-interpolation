# quaternion-rotation-interpolation

A small ESM library that performs spherical linear interpolation (slerp) between unit quaternions and composes rotation sequences for 3D orientations.

```js
import { slerp, compose, fromAxisAngle, multiply, conjugate, normalize } from 'quaternion-rotation-interpolation';

// 90° about x, then 90° about y, applied in written order
const q1 = fromAxisAngle([1, 0, 0], Math.PI / 2);
const q2 = fromAxisAngle([0, 1, 0], Math.PI / 2);
const combined = compose([q1, q2]);

// interpolate halfway from identity to `combined`
const halfway = slerp([0, 0, 0, 1], combined, 0.5);

// invert a rotation
const inverse = conjugate(combined);
```

Quaternions are `[x, y, z, w]`. Multiplication uses the Hamilton product. `compose([q1, q2, q3])` returns `q3 * q2 * q1`, so a vector rotated by the result is first rotated by `q1`, then `q2`, then `q3` — the written order is the application order.

`slerp` normalizes its inputs once at the start and always returns a unit quaternion. The interpolation takes the shorter arc: when two rotations are nearly opposite (their 4-vectors point away from each other), one operand is negated so the path stays under 180°. For antipodal inputs, where the shortest arc is undefined, the implementation chooses a fixed direction by negating the second operand rather than throwing — callers that need a specific direction should detect this case and pass an explicit end.

The library exists because correct slerp has two quietly awkward edges — the near-parallel case (where dividing by `sin(theta)` loses significant digits) and the antiparallel case (where the shortest arc is undefined) — and handling both inline keeps a rotation pipeline self-contained without pulling a math framework. The trade-off is that `compose` left-folds and returns a unit quaternion: it does not preserve arbitrary scale or support non-rotation (dual) quaternions.
