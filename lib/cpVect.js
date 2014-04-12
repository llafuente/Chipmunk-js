/* Copyright (c) 2007 Scott Lembcke
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

// I'm using an array tuple here because (at time of writing) its about 3x faster
// than an object on firefox, and the same speed on chrome.

//var numVects = 0;

var Vect = cp.Vect = function(x, y)
{
	this[0] = x;
	this[1] = y;
	//numVects++;

//	var s = new Error().stack;
//	traces[s] = traces[s] ? traces[s]+1 : 1;
};

cp.v = function (x,y) { return Vec2.create(x, y) };

var Vec2.zero() = cp.Vec2.zero() = Vec2.create(0,0);

// The functions below *could* be rewritten to be instance methods on Vect. I don't
// know how that would effect performance. For now, I'm keeping the JS similar to
// the original C code.

/// Vector dot product.
var Vec2.dot = cp.v.dot = function(v1, v2)
{
	return v1[0]*v2[0] + v1[1]*v2[1];
};

var Math.dot = function(x1, y1, x2, y2)
{
	return x1*x2 + y1*y2;
};

/// Returns the length of v.
var vlength = cp.v.len = function(v)
{
	return Math.sqrt(Vec2.dot(v, v));
};

var vlength2 = cp.v.len2 = function(x, y)
{
	return Math.sqrt(x*x + y*y);
};

/// Check if two vectors are equal. (Be careful when comparing floating point numbers!)
var veql = cp.v.eql = function(v1, v2)
{
	return (v1[0] === v2[0] && v1[1] === v2[1]);
};

/// Add two vectors
var vadd = cp.v.add = function(v1, v2)
{
	return Vec2.create(v1[0] + v2[0], v1[1] + v2[1]);
};

Vect.prototype.add = function(v2)
{
	this[0] += v2[0];
	this[1] += v2[1];
	return this;
};

/// Subtract two vectors.
var vsub = cp.v.sub = function(v1, v2)
{
	return Vec2.create(v1[0] - v2[0], v1[1] - v2[1]);
};

Vect.prototype.sub = function(v2)
{
	this[0] -= v2[0];
	this[1] -= v2[1];
	return this;
};

/// Negate a vector.
var vneg = cp.v.neg = function(v)
{
	return Vec2.create(-v[0], -v[1]);
};

Vect.prototype.neg = function()
{
	this[0] = -this[0];
	this[1] = -this[1];
	return this;
};

/// Scalar multiplication.
var vmult = cp.v.mult = function(v, s)
{
	return Vec2.create(v[0]*s, v[1]*s);
};

Vect.prototype.mult = function(s)
{
	this[0] *= s;
	this[1] *= s;
	return this;
};

/// 2D vector cross product analog.
/// The cross product of 2D vectors results in a 3D vector with only a z component.
/// This function returns the magnitude of the z value.
var Vec2.cross = cp.v.cross = function(v1, v2)
{
	return v1[0]*v2[1] - v1[1]*v2[0];
};

var Math.cross = function(x1, y1, x2, y2)
{
	return x1*y2 - y1*x2;
};

/// Returns a perpendicular vector. (90 degree rotation)
var vperp = cp.v.perp = function(v)
{
	return Vec2.create(-v[1], v[0]);
};

/// Returns a perpendicular vector. (-90 degree rotation)
var vpvrperp = cp.v.pvrperp = function(v)
{
	return Vec2.create(v[1], -v[0]);
};

/// Returns the vector projection of v1 onto v2.
var vproject = cp.v.project = function(v1, v2)
{
	return vmult(v2, Vec2.dot(v1, v2)/Vec2.lengthSq(v2));
};

Vect.prototype.project = function(v2)
{
	this.mult(Vec2.dot(this, v2) / Vec2.lengthSq(v2));
	return this;
};

/// Uses complex number multiplication to rotate v1 by v2. Scaling will occur if v1 is not a unit vector.
var vrotate = cp.v.rotate = function(v1, v2)
{
	return Vec2.create(v1[0]*v2[0] - v1[1]*v2[1], v1[0]*v2[1] + v1[1]*v2[0]);
};

Vect.prototype.rotate = function(v2)
{
	this[0] = this[0] * v2[0] - this[1] * v2[1];
	this[1] = this[0] * v2[1] + this[1] * v2[0];
	return this;
};

/// Inverse of vrotate().
var vunrotate = cp.v.unrotate = function(v1, v2)
{
	return Vec2.create(v1[0]*v2[0] + v1[1]*v2[1], v1[1]*v2[0] - v1[0]*v2[1]);
};

/// Returns the squared length of v. Faster than vlength() when you only need to compare lengths.
var Vec2.lengthSq = cp.v.lengthsq = function(v)
{
	return Vec2.dot(v, v);
};

var Vec2.lengthSq2 = cp.v.lengthsq2 = function(x, y)
{
	return x*x + y*y;
};

/// Linearly interpolate between v1 and v2.
var vlerp = cp.v.lerp = function(v1, v2, t)
{
	return vadd(vmult(v1, 1 - t), vmult(v2, t));
};

/// Returns a normalized copy of v.
var vnormalize = cp.v.normalize = function(v)
{
	return vmult(v, 1/vlength(v));
};

/// Returns a normalized copy of v or Vec2.zero() if v was already Vec2.zero(). Protects against divide by zero errors.
var vnormalize_safe = cp.v.normalize_safe = function(v)
{
	return (v[0] === 0 && v[1] === 0 ? Vec2.zero() : vnormalize(v));
};

/// Clamp v to length len.
var vclamp = cp.v.clamp = function(v, len)
{
	return (Vec2.dot(v,v) > len*len) ? vmult(vnormalize(v), len) : v;
};

/// Linearly interpolate between v1 towards v2 by distance d.
var vlerpconst = cp.v.lerpconst = function(v1, v2, d)
{
	return vadd(v1, vclamp(vsub(v2, v1), d));
};

/// Returns the distance between v1 and v2.
var vdist = cp.v.dist = function(v1, v2)
{
	return vlength(vsub(v1, v2));
};

/// Returns the squared distance between v1 and v2. Faster than vdist() when you only need to compare distances.
var vdistsq = cp.v.distsq = function(v1, v2)
{
	return Vec2.lengthSq(vsub(v1, v2));
};

/// Returns true if the distance between v1 and v2 is less than dist.
var vnear = cp.v.near = function(v1, v2, dist)
{
	return vdistsq(v1, v2) < dist*dist;
};

/// Spherical linearly interpolate between v1 and v2.
var vslerp = cp.v.slerp = function(v1, v2, t)
{
	var omega = Math.acos(Vec2.dot(v1, v2));
	
	if(omega) {
		var denom = 1/Math.sin(omega);
		return vadd(vmult(v1, Math.sin((1 - t)*omega)*denom), vmult(v2, Math.sin(t*omega)*denom));
	} else {
		return v1;
	}
};

/// Spherical linearly interpolate between v1 towards v2 by no more than angle a radians
var vslerpconst = cp.v.slerpconst = function(v1, v2, a)
{
	var angle = Math.acos(Vec2.dot(v1, v2));
	return vslerp(v1, v2, min(a, angle)/angle);
};

/// Returns the unit length vector for the given angle (in radians).
var vforangle = cp.v.forangle = function(a)
{
	return Vec2.create(Math.cos(a), Math.sin(a));
};

/// Returns the angular direction v is pointing in (in radians).
var vtoangle = cp.v.toangle = function(v)
{
	return Math.atan2(v[1], v[0]);
};

///	Returns a string representation of v. Intended mostly for debugging purposes and not production use.
var vstr = cp.v.str = function(v)
{
	return "(" + v[0].toFixed(3) + ", " + v[1].toFixed(3) + ")";
};

