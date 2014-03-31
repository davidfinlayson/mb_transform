mb_transform
============

A small 3D transformation library for MB System

A simple coodinate transformation library for MB System. This library is
designed to perform the simple affine transformations needed during the
routine processing of multibeam data. It supports ridged translations and
rotations about an arbitrary origin and includes a SLERP function for
smoothly interpolating orientations.

The library (currently) uses a left-handed coordinate system with the
following axis:

- x-axis positive to the right
- y-axis positive up
- z-axis positive forward

- Rotation about the x-axis (pitch) is positive nose down
- Rotation about the y-axis (heading) is positive nose right
- Rotation about the z-axis (bank) is positive starboard up

The library is based on a 4x3 transformation matrix (mb_matrix) which can
be computed very efficiently. Rotations and translations can be applied in
arbitrary order which is necessary to support sonars which do not use
the heading-pitch-bank system (aka roll-pitch-yaw).

Adapted from:
-------------

Dunn, Fletcher and Parberry, Ian (2002) 3D Math Primer for Graphics
and Game Development. WordWare Publishing, Inc., Sudbury, MA

Mak, Ronald (2003) The Java Programmer's Guide to Numerical Computing.
Prentice Hall PTR, Upper Saddle River, NJ
