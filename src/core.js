/**
 * Quaternion math for 3D rotation. Quaternions are [x, y, z, w], where
 * x, y, z are the imaginary part and w the real (scalar) part.
 *
 * Convention: left-to-right application in the public helpers. compose([q1, q2])
 * returns q2 * q1, so a vector rotated by the result is first rotated by q1,
 * then by q2. This is the intuitive reading of a written rotation order.
 */

/**
 * Componentwise sum.
 */
export function add(a, b) {
  return [a[0] + b[0], a[1] + b[1], a[2] + b[2], a[3] + b[3]];
}

/**
 * Componentwise scalar product.
 */
export function scale(a, s) {
  return [a[0] * s, a[1] * s, a[2] * s, a[3] * s];
}

/**
 * Dot product of two quaternions (4-vector).
 */
export function dot(a, b) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
}

/**
 * Euclidean norm (length).
 */
export function length(a) {
  return Math.sqrt(dot(a, a));
}

/**
 * Normalize to unit length. Returns [0,0,0,1] when the input is the zero
 * quaternion, which has no meaningful direction.
 */
export function normalize(a) {
  const len = length(a);
  if (len === 0) return [0, 0, 0, 1];
  return [a[0] / len, a[1] / len, a[2] / len, a[3] / len];
}

/**
 * Hamilton product. The quaternion multiplication operator, a * b.
 */
export function multiply(a, b) {
  const ax = a[0], ay = a[1], az = a[2], aw = a[3];
  const bx = b[0], by = b[1], bz = b[2], bw = b[3];

  return [
    aw * bx + ax * bw + ay * bz - az * by,
    aw * by - ax * bz + ay * bw + az * bx,
    aw * bz + ax * by - ay * bx + az * bw,
    aw * bw - ax * bx - ay * by - az * bz,
  ];
}

/**
 * Negate (additive inverse).
 */
export function negate(a) {
  return [-a[0], -a[1], -a[2], -a[3]];
}

/**
 * Multiplicative conjugate: [x, y, z, -w]. For a unit quaternion this is the
 * inverse, which represents the opposite rotation.
 */
export function conjugate(a) {
  return [-a[0], -a[1], -a[2], a[3]];
}

/**
 * Build a unit quaternion for a rotation of `angle` radians about an `axis`.
 * The axis need not be unit length; it is normalized internally.
 */
export function fromAxisAngle(axis, angle) {
  const [x, y, z] = axis;
  const n = Math.sqrt(x * x + y * y + z * z);
  if (n === 0) return [0, 0, 0, 1];

  const inv = 1 / n;
  const h = angle / 2;
  const s = Math.sin(h);

  return [
    x * inv * s,
    y * inv * s,
    z * inv * s,
    Math.cos(h),
  ];
}

/**
 * Shortest-arc spherical linear interpolation between two unit quaternions.
 *
 * For inputs that are not exactly unit length, each is normalized once at the
 * start rather than per-step; this is O(1) overhead and avoids pathological
 * drift in long sequences. The interpolation itself then preserves length.
 *
 * Near-parallel and anti-parallel inputs are the awkward case. For cos(theta)
 * above ~0.9995 we switch to linear interpolation plus normalization (nlerp):
 * at that scale, dividing by sin(theta) loses five significant digits.
 * Antipodal inputs pick a fixed direction by negating q0, because the
 * zero-length shortest arc is undefined.
 *
 * Returns a unit quaternion.
 */
export function slerp(q0, q1, t) {
  const a = normalize(q0);
  const b = normalize(q1);

  let cosT = dot(a, b);
  let end = b;

  // Take the shorter arc: if the angle between the 4-vectors exceeds 90°,
  // negate one operand. This flips the rotation direction to the complement
  // (e.g. rotating 270° clockwise becomes 90° counterclockwise).
  if (cosT < 0) {
    end = negate(b);
    cosT = -cosT;
  }

  // cosT is in [0, 1] from here on.
  const EPSILON = 1e-9;

  if (cosT > 1 - EPSILON) {
    // Inputs are parallel; sin(theta) is near zero, so skip the division.
    return normalize(add(a, scale(end, t)));
  }

  const theta = Math.acos(cosT);
  const sinTheta = Math.sin(theta);
  const w0 = Math.sin((1 - t) * theta) / sinTheta;
  const w1 = Math.sin(t * theta) / sinTheta;

  return [
    a[0] * w0 + end[0] * w1,
    a[1] * w0 + end[1] * w1,
    a[2] * w0 + end[2] * w1,
    a[3] * w0 + end[3] * w1,
  ];
}

/**
 * Compose a rotation sequence. compose([q1, q2, q3]) returns q3 * q2 * q1, so
 * when applied to a vector the rotations happen in written order: q1, q2,
 * then q3. This is left-to-right application and matches how rotation orders
 * are typically specified. Returns a unit quaternion.
 */
export function compose(quaternions) {
  if (quaternions.length === 0) return [0, 0, 0, 1];
  let acc = normalize(quaternions[0]);
  for (let i = 1; i < quaternions.length; i++) {
    acc = multiply(normalize(quaternions[i]), acc);
  }
  return acc;
}
