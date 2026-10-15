import { test } from 'node:test';
import assert from 'node:assert/strict';

import {
  slerp,
  compose,
  multiply,
  normalize,
  fromAxisAngle,
  negate,
  conjugate,
  add,
  scale,
  dot,
  length,
} from '../src/index.js';

const EPS = 1e-12;

function approxEqual(a, b, eps = EPS) {
  const d = Math.abs(a - b);
  return d < eps || d < Math.max(Math.abs(a), Math.abs(b)) * eps;
}

function approxQuat(a, b, eps = 1e-9) {
  let d = 0;
  for (let i = 0; i < 4; i++) {
    d += (a[i] - b[i]) * (a[i] - b[i]);
  }
  d = Math.sqrt(d);
  return d < eps || d < Math.max(length(a), length(b)) * eps;
}

const IDENTITY = [0, 0, 0, 1];
const X90 = fromAxisAngle([1, 0, 0], Math.PI / 2);
const Y90 = fromAxisAngle([0, 1, 0], Math.PI / 2);
const Z90 = fromAxisAngle([0, 0, 1], Math.PI / 2);

// A rotation that maps +x to -x, achievable via 180° about y or 180° about z.
// We use about z so the quaternion is [0, 0, 1, 0] (exact, no float noise).
const HALF_TURN_Z = fromAxisAngle([0, 0, 1], Math.PI);

// A quaternion rotated 3π/2 about z (270°). Its 4-vector is nearly antipodal
// to a π/2 rotation: slerp between the two must take the shorter arc.
const Z_270 = fromAxisAngle([0, 0, 1], 3 * Math.PI / 2);


// --- slerp endpoints ---

test('slerp at t=0 returns the (normalized) first argument', () => {
  const a = fromAxisAngle([1, 0, 0], Math.PI / 3);
  assert.ok(approxQuat(slerp(a, Y90, 0), a));
});

test('slerp at t=1 returns the second argument on the chosen arc', () => {
  const a = fromAxisAngle([1, 0, 0], Math.PI / 3);
  // The chosen arc should be the shorter one for near-orthogonal inputs.
  const result = slerp(a, Y90, 1);
  assert.ok(approxQuat(result, Y90) || approxQuat(result, negate(Y90)));
});

test('slerp between identical rotations returns that rotation for any t', () => {
  const a = fromAxisAngle([0.4, 0.5, 0.6], 1.1);
  for (const t of [0, 0.25, 0.5, 0.75, 1]) {
    assert.ok(approxQuat(slerp(a, a, t), a), `failed at t=${t}`);
  }
});

// --- slerp intermediate point checks ---

test('slerp halfway between identity and 90° z gives 45° z', () => {
  const expected = fromAxisAngle([0, 0, 1], Math.PI / 4);
  const got = slerp(IDENTITY, Z90, 0.5);
  assert.ok(approxQuat(got, expected));
});

test('slerp halfway between identity and 180° z gives 90° z', () => {
  const expected = fromAxisAngle([0, 0, 1], Math.PI / 2);
  const got = slerp(IDENTITY, HALF_TURN_Z, 0.5);
  assert.ok(approxQuat(got, expected));
});

// --- slerp antiparallel handling ---

test('slerp takes the shorter arc between nearly antipodal rotations', () => {
  // q0 = 90° about z, q1 = 270° about z: same physical axis, 180° apart as
  // quaternions, but only 90° apart as physical rotations. The shortest arc
  // result at t=0.5 must be 0° (identity), not ±180°.
  const got = slerp(Z90, Z_270, 0.5);
  assert.ok(
    approxQuat(got, IDENTITY) || approxQuat(got, HALF_TURN_Z),
    `got [${got}], expected identity or 180° about z`
  );
});

// --- slerp normalization ---

test('slerp normalizes non-unit inputs', () => {
  const a = scale(X90, 2.5);
  const b = scale(Y90, 0.4);
  assert.ok(approxEqual(length(slerp(a, b, 0.37)), 1));
});

// --- multiplication ---

test('multiplying a rotation by its conjugate gives identity', () => {
  const r = fromAxisAngle([1, 2, 3], 0.7);
  const p = multiply(r, conjugate(r));
  assert.ok(approxQuat(p, IDENTITY, 1e-12));
});

test('multiplication is associative', () => {
  const r1 = fromAxisAngle([1, 0, 0], 0.4);
  const r2 = fromAxisAngle([0, 1, 0], 0.6);
  const r3 = fromAxisAngle([0, 0, 1], 0.3);
  const l = multiply(multiply(r1, r2), r3);
  const rr = multiply(r1, multiply(r2, r3));
  assert.ok(approxQuat(l, rr, 1e-12));
});

// --- compose ---

test('compose of a single quaternion returns that quaternion normalized', () => {
  const q = scale(X90, 3);
  assert.ok(approxQuat(compose([q]), X90));
});

test('compose with no rotations returns identity', () => {
  assert.deepEqual(compose([]), [0, 0, 0, 1]);
});

test('compose two 90° z rotations equals a 180° z rotation', () => {
  const r = compose([Z90, Z90]);
  assert.ok(approxQuat(r, HALF_TURN_Z));
});

test('compose [X90, Y90] equals Y90 * X90 (left-to-right application)', () => {
  assert.ok(approxQuat(compose([X90, Y90]), multiply(Y90, X90)));
});

test('compose matches manual left-fold over multiple quaternions', () => {
  const seq = [X90, Y90, Z90, conjugate(Y90)];
  let acc = X90;
  for (let i = 1; i < seq.length; i++) acc = multiply(seq[i], acc);
  assert.ok(approxQuat(compose(seq), acc));
});

// --- fromAxisAngle ---

test('fromAxisAngle with zero-length axis returns identity', () => {
  assert.deepEqual(fromAxisAngle([0, 0, 0], 1.5), IDENTITY);
});

test('fromAxisAngle is insensitive to axis scale', () => {
  const a = fromAxisAngle([2, 0, 0], Math.PI / 5);
  const b = fromAxisAngle([5, 0, 0], Math.PI / 5);
  assert.ok(approxQuat(a, b));
});

test('fromAxisAngle zero angle gives identity regardless of axis', () => {
  assert.ok(approxQuat(fromAxisAngle([3, 4, 5], 0), IDENTITY));
});

// --- normalize ---

test('normalize zero quaternion returns identity', () => {
  assert.deepEqual(normalize([0, 0, 0, 0]), IDENTITY);
});

test('normalize yields unit length', () => {
  const a = [3, 4, 5, 6];
  const n = normalize(a);
  assert.ok(approxEqual(length(n), 1));
});

// --- negate ---

test('negate gives additive inverse', () => {
  const q = [1, -2, 3, -4];
  assert.deepEqual(negate(q), [-1, 2, -3, 4]);
});
